`timescale 1ns/1ps

interface controller_intf(input logic clk);
    logic rst_n, start, rw, busy, done, nack_detected;
    logic [6:0] address;
    logic [7:0] data, data_rcvd;
    
    modport DUT(
        input clk, rst_n, start, rw, address, data,
        output busy, done, nack_detected, data_rcvd
    );
    modport driver(
        input clk, busy, done, nack_detected, data_rcvd,
        output rst_n, start, rw, address, data
    );
    modport monitor(
        input clk, rst_n, start, rw, address, data, busy, done, nack_detected, data_rcvd
    );
endinterface

class transaction;
    rand bit rw;
    rand bit [6:0] address;
    rand bit [7:0] data;
    bit rst_n;
endclass

class generator;
    mailbox #(transaction) to_driver;
    
    function new(mailbox #(transaction) mb);
        to_driver = mb;
    endfunction
    
    task send_command(
        input bit [6:0] address,
        input bit rw,
        input bit [7:0] data
    );
        transaction tr;
        tr = new();
        tr.rst_n = 1'b1;
        tr.address = address;
        tr.rw = rw;
        tr.data = data;
        to_driver.put(tr);
    endtask
    
    task run(input int n_random);
        transaction tr;
        tr = new();
        tr.rst_n = 1'b0;
        to_driver.put(tr);
        
        send_command(7'h42, 1'b0, 8'h00);
        send_command(7'h42, 1'b0, 8'hFF);
        send_command(7'h42, 1'b1, 8'h00);
        send_command(7'h43, 1'b0, 8'h55);
        
        for(int i=0; i<n_random; i++) begin
            tr = new();
            tr.rst_n = 1'b1;
            if(i%2==0) begin
                if(!tr.randomize() with {address == 7'h42;})
                    $fatal(1, "Randomization failed");
            end
            else begin
                if(!tr.randomize() with {
                    address inside {[7'h08:7'h77]};
                    address != 7'h42;
                })
                    $fatal(1, "Randomization failed");
            end
            to_driver.put(tr);
        end
    endtask
endclass

class driver;
    mailbox #(transaction) from_generator;
    virtual controller_intf.driver vif;
    int unsigned max_wait_cycles;
    
    function new(
        mailbox #(transaction) mb,
        virtual controller_intf.driver vif,
        input int unsigned max_wait_cycles=100000
    );
        from_generator = mb;
        this.vif = vif;
        this.max_wait_cycles = max_wait_cycles;
    endfunction
    
    task run();
        transaction tr;
        int unsigned cycles;
        vif.rst_n = 1'b0;
        vif.start = 1'b0;
        vif.rw = 1'b0;
        vif.address = 7'b0;
        vif.data = 8'b0;
        
        forever begin
            from_generator.get(tr);
            if(!tr.rst_n) begin
                @(negedge vif.clk);
                vif.start = 1'b0;
                vif.rst_n = 1'b0;
                repeat(3) @(negedge vif.clk);
                vif.rst_n = 1'b1;
            end
            else begin
                @(negedge vif.clk);
                cycles = 0;
                while(vif.busy !== 1'b0) begin
                    if(cycles >= max_wait_cycles)
                        $fatal(1, "Driver: timeout waiting for idle");
                    @(negedge vif.clk);
                    cycles++;
                end
                
                vif.address = tr.address;
                vif.rw = tr.rw;
                vif.data = tr.data;
                vif.start = 1'b1;
                
                @(negedge vif.clk);
                vif.start = 1'b0;
                if(vif.busy !== 1'b1)
                    $fatal(1, "Driver: DUT did not accept command");
                
                cycles = 0;
                while(vif.done !== 1'b1) begin
                    if(cycles >= max_wait_cycles)
                        $fatal(1, "Driver: timeout waiting for done");
                    @(negedge vif.clk);
                    cycles++;
                end
            end
        end
    endtask
endclass

class observation;
    logic rw;
    logic [6:0] address;
    logic [7:0] data;
    logic [7:0] data_rcvd;
    logic nack_detected;
    logic busy_at_done;
endclass

class monitor;
    virtual controller_intf.monitor vif;
    mailbox #(observation) to_scoreboard;
    
    function new(
        virtual controller_intf.monitor vif,
        mailbox #(observation) mb
    );
        this.vif = vif;
        to_scoreboard = mb;
    endfunction
    
    task run();
        observation obs;
        forever begin
            @(posedge vif.clk);
            if(vif.rst_n === 1'b1 && vif.start === 1'b1 && vif.busy === 1'b0) begin
                obs = new();
                obs.address = vif.address;
                obs.rw = vif.rw;
                obs.data = vif.data;
                
                @(posedge vif.clk);
                if(vif.rst_n !== 1'b1)
                    continue;
                if(vif.busy !== 1'b1)
                    $fatal(1, "Monitor: DUT did not accept command");
                
                while(vif.done !== 1'b1) begin
                    @(posedge vif.clk);
                    if(vif.rst_n !== 1'b1)
                        break;
                end
                
                if(vif.rst_n === 1'b1) begin
                    obs.data_rcvd = vif.data_rcvd;
                    obs.nack_detected = vif.nack_detected;
                    obs.busy_at_done = vif.busy;
                    to_scoreboard.put(obs);
                end
            end
        end
    endtask
endclass

class bus_observation;
    logic samples[$];
endclass

interface i2c_intf(input logic rst_n);
    tri1 scl_bus, sda_bus;
    mailbox #(bus_observation) to_scoreboard = new();
    bus_observation current_transfer;
    bit active = 1'b0;
    
    always @(negedge sda_bus) begin
        if(rst_n === 1'b1 && scl_bus === 1'b1) begin
            if(active)
                $fatal(1, "Bus monitor: unexpected repeated START");
            current_transfer = new();
            active = 1'b1;
        end
    end
    
    always @(posedge scl_bus) begin
        if(rst_n === 1'b1 && active)
            current_transfer.samples.push_back(sda_bus);
    end
    
    always @(posedge sda_bus) begin
        if(rst_n === 1'b1 && scl_bus === 1'b1 && active) begin
            if(current_transfer.samples.size()==0)
                $fatal(1, "Bus monitor: STOP without SCL samples");
            if(current_transfer.samples.pop_back() !== 1'b0)
                $fatal(1, "Bus monitor: invalid STOP preparation");
            to_scoreboard.put(current_transfer);
            active = 1'b0;
        end
    end
    
    always @(negedge rst_n)
        active = 1'b0;
endinterface

module i2c_target #(
    parameter logic [6:0] TARGET_ADDRESS = 7'h42,
    parameter logic [7:0] READ_DATA = 8'hA6
)(
    input logic rst_n,
    input wire scl_bus,
    inout wire sda_bus
);
    bit sda_low = 1'b0;
    logic [7:0] address_byte;
    logic [7:0] write_byte;
    assign sda_bus = sda_low ? 1'b0 : 1'bz;
    
    task automatic wait_start();
        forever begin
            @(negedge sda_bus);
            if(scl_bus === 1'b1)
                return;
        end
    endtask
    
    task automatic receive_byte(output logic [7:0] value);
        for(int i=7; i>=0; i--) begin
            @(posedge scl_bus);
            value[i] = sda_bus;
        end
    endtask
    
    task automatic send_ack();
        @(negedge scl_bus);
        sda_low = 1'b1;
        @(posedge scl_bus);
        @(negedge scl_bus);
        sda_low = 1'b0;
    endtask
    
    task automatic send_byte(input logic [7:0] value);
        for(int i=7; i>=0; i--) begin
            sda_low = ~value[i];
            @(posedge scl_bus);
            @(negedge scl_bus);
        end
        sda_low = 1'b0;
        @(posedge scl_bus);
        @(negedge scl_bus);
    endtask
    
    task automatic wait_stop();
        forever begin
            @(posedge sda_bus);
            if(scl_bus === 1'b1)
                return;
        end
    endtask
    
    task automatic serve_transfers();
        forever begin
            wait_start();
            receive_byte(address_byte);
            if(address_byte[7:1] === TARGET_ADDRESS) begin
                send_ack();
                if(address_byte[0] === 1'b0) begin
                    receive_byte(write_byte);
                    send_ack();
                end
                else
                    send_byte(READ_DATA);
            end
            wait_stop();
        end
    endtask
    
    initial begin
        forever begin
            wait(rst_n === 1'b1);
            fork : target_activity
                serve_transfers();
                begin
                    wait(rst_n === 1'b0);
                end
            join_any
            disable target_activity;
            sda_low = 1'b0;
        end
    end
endmodule

class scoreboard;
    mailbox #(observation) from_monitor;
    mailbox #(bus_observation) from_bus;
    bit [6:0] target_address;
    bit [7:0] target_read_data;
    int unsigned pass, fail;
    
    function new(
        mailbox #(observation) status_mb,
        mailbox #(bus_observation) bus_mb,
        input bit [6:0] target_address = 7'h42,
        input bit [7:0] target_read_data = 8'hA6
    );
        from_monitor = status_mb;
        from_bus = bus_mb;
        this.target_address = target_address;
        this.target_read_data = target_read_data;
        pass = 0;
        fail = 0;
    endfunction
    
    task run(input int n);
        observation obs;
        bus_observation bus_obs;
        bit expected_nack;
        bit [7:0] expected_read_data;
        bit result_ok;
        logic [7:0] bus_address_byte;
        logic [7:0] bus_data_byte;
        int expected_samples;
        
        repeat(n) begin
            from_monitor.get(obs);
            from_bus.get(bus_obs);
            result_ok = 1'b1;
            expected_nack = (obs.address !== target_address);
            expected_read_data = 8'h00;
            
            if(!expected_nack && obs.rw === 1'b1)
                expected_read_data = target_read_data;
            if($isunknown({obs.address, obs.rw, obs.data})) begin
                $error("Scoreboard: X/Z detected in command");
                result_ok = 1'b0;
            end
            if(obs.nack_detected !== expected_nack) begin
                $error("NACK mismatch. address = %h expected nack status = %b actual = %b",
                       obs.address, expected_nack, obs.nack_detected);
                result_ok = 1'b0;
            end
            if(obs.data_rcvd !== expected_read_data) begin
                $error("Read data mismatch. Expected = %h, actual = %h",
                       expected_read_data, obs.data_rcvd);
                result_ok = 1'b0;
            end
            if(obs.busy_at_done !== 1'b0) begin
                $error("Scoreboard: busy must be LOW when done is HIGH");
                result_ok = 1'b0;
            end
            
            expected_samples = expected_nack ? 9 : 18;
            if(bus_obs.samples.size()!=expected_samples) begin
                $error("Bus length mismatch. Expected = %0d, actual = %0d",
                       expected_samples, bus_obs.samples.size());
                result_ok = 1'b0;
            end
            else begin
                for(int i=0; i<8; i++)
                    bus_address_byte[7-i] = bus_obs.samples[i];
                
                if(bus_address_byte !== {obs.address, obs.rw}) begin
                    $error("Bus address/RW mismatch. Expected = %h, actual = %h",
                           {obs.address, obs.rw}, bus_address_byte);
                    result_ok = 1'b0;
                end
                if(bus_obs.samples[8] !== expected_nack) begin
                    $error("Bus address ACK/NACK mismatch");
                    result_ok = 1'b0;
                end
                
                if(!expected_nack) begin
                    for(int i=0; i<8; i++)
                        bus_data_byte[7-i] = bus_obs.samples[9+i];
                    
                    if(obs.rw === 1'b0) begin
                        if(bus_data_byte !== obs.data) begin
                            $error("Bus write mismatch. Expected = %h, actual = %h",
                                   obs.data, bus_data_byte);
                            result_ok = 1'b0;
                        end
                        if(bus_obs.samples[17] !== 1'b0) begin
                            $error("Bus: target did not ACK write data");
                            result_ok = 1'b0;
                        end
                    end
                    else begin
                        if(bus_data_byte !== target_read_data) begin
                            $error("Bus read mismatch. Expected = %h, actual = %h",
                                   target_read_data, bus_data_byte);
                            result_ok = 1'b0;
                        end
                        if(bus_obs.samples[17] !== 1'b1) begin
                            $error("Bus: controller did not NACK final read byte");
                            result_ok = 1'b0;
                        end
                    end
                end
            end
            
            if(result_ok) begin
                pass++;
                $display("[SCB] PASS: address = %h, rw = %b, write_data = %h, read_data = %h, nack = %b",
                         obs.address, obs.rw, obs.data, obs.data_rcvd, obs.nack_detected);
            end
            else
                fail++;
        end
        
        $display("[SCB] Tests Passed = %0d; failed = %0d", pass, fail);
        if(fail!=0)
            $fatal(1, "[SCB] Design failure detected");
    endtask
endclass

module i2c_tb_selfcheck;
    localparam int PHASE_CYCLES = 4;
    localparam int N_RANDOM = 20;
    localparam int N_TRANSFERS = 4 + N_RANDOM;
    localparam bit [6:0] TARGET_ADDRESS = 7'h42;
    localparam bit [7:0] TARGET_READ_DATA = 8'hA6;
    
    logic clk = 1'b0;
    always #5 clk = ~clk;
    
    controller_intf intf(clk);
    i2c_intf bus(intf.rst_n);
    
    i2c_controller #(
        .phase_cycles(PHASE_CYCLES)
    ) dut(
        .clk(clk),
        .rst_n(intf.rst_n),
        .start(intf.start),
        .rw(intf.rw),
        .address(intf.address),
        .data(intf.data),
        .busy(intf.busy),
        .done(intf.done),
        .nack_detected(intf.nack_detected),
        .data_rcvd(intf.data_rcvd),
        .scl_bus(bus.scl_bus),
        .sda_bus(bus.sda_bus)
    );
    
    i2c_target #(
        .TARGET_ADDRESS(TARGET_ADDRESS),
        .READ_DATA(TARGET_READ_DATA)
    ) target(
        .rst_n(intf.rst_n),
        .scl_bus(bus.scl_bus),
        .sda_bus(bus.sda_bus)
    );
    
    mailbox #(transaction) gen_to_drv;
    mailbox #(observation) mon_to_scb;
    generator gen;
    driver drv;
    monitor mon;
    scoreboard scb;
    
    initial begin
        gen_to_drv = new();
        mon_to_scb = new();
        gen = new(gen_to_drv);
        drv = new(gen_to_drv, intf);
        mon = new(intf, mon_to_scb);
        scb = new(mon_to_scb, bus.to_scoreboard, TARGET_ADDRESS, TARGET_READ_DATA);
        
        fork
            gen.run(N_RANDOM);
            drv.run();
            mon.run();
            begin
                scb.run(N_TRANSFERS);
                $display("All requested transfers checked");
                $finish;
            end
        join
    end
    
    initial begin
        repeat((N_TRANSFERS+1)*100000) @(posedge clk);
        $fatal(1, "Overall testbench timeout");
    end
endmodule