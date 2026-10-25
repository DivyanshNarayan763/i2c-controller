module i2c_controller #(parameter int phase_cycles=250)(
    input logic clk, rst_n, start, rw,
    input logic [6:0]address,
    input logic [7:0]data,
    output logic busy, done, nack_detected,
    output logic [7:0]data_rcvd,
    inout tri1 scl_bus, sda_bus
);

typedef enum logic [4:0]{
    IDLE, START_HOLD, START_LOW, BIT_SETUP, BIT_HIGH, BIT_FALL,
    ACK_SETUP, ACK_HIGH, ACK_FALL, STOP_LOW, STOP_HIGH, STOP_RELEASE,
    RX_SETUP, RX_HIGH, RX_FALL, RX_NACK_SETUP, RX_NACK_HIGH
}state_t;

logic scl_low, sda_low;
assign scl_bus = scl_low ? 1'b0 : 1'bz;
assign sda_bus = sda_low ? 1'b0 : 1'bz;

logic rw_latched;
logic [7:0] wr_data_latched;

state_t state;
logic [7:0] shift_reg;
logic [2:0] index;
logic sending_data_byte;
logic target_acked;
int unsigned count;

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state <= IDLE;
        {scl_low, sda_low} <= 2'b0;
        {busy, done} <= 2'b0;
        nack_detected <= 1'b0;
        shift_reg <= 8'h0;
        index <= 3'b111;
        target_acked <= 1'b0;
        count <= 0;
        sending_data_byte <= 1'b0;
        rw_latched <= 1'b0;
        wr_data_latched <= 8'b0;
        data_rcvd <= 8'b0;
    end
    else begin
        done <= 1'b0;
        case(state)
            IDLE: begin
                {scl_low, sda_low} <= 2'b0;
                busy <= 1'b0;
                if(start && scl_bus == 1'b1 && sda_bus == 1'b1) begin
                    data_rcvd <= 8'b0;
                    busy <= 1'b1;
                    nack_detected <= 1'b0;
                    shift_reg <= {address, rw};
                    rw_latched <= rw;
                    index <= 3'b111;
                    sda_low <= 1'b1;
                    state <= START_HOLD;
                    count <= 0;
                    sending_data_byte <= 1'b0;
                    target_acked <= 1'b0;
                    wr_data_latched <= data;
                end
            end
            START_HOLD: begin
                if(count == phase_cycles-1) begin
                    count <= 0;
                    scl_low <= 1'b1;
                    state <= START_LOW;
                end
                else count <= count + 1;
            end
            START_LOW: begin
                if(count == phase_cycles-1) begin
                    count <= 0;
                    state <= BIT_SETUP;
                end
                else count <= count + 1;
            end
            BIT_SETUP: begin
                sda_low <= shift_reg[7] ? 1'b0 : 1'b1;

                if(count == phase_cycles-1) begin
                    count <= 0;
                    scl_low <= 1'b0;
                    state <= BIT_HIGH;
                end
                else count <= count + 1;
            end
            BIT_HIGH: begin
                if(scl_bus) begin
                    if(count == phase_cycles-1) begin
                        count <= 0;
                        scl_low <= 1'b1;
                        state <= BIT_FALL;
                    end
                    else count <= count + 1;
                end
            end
            BIT_FALL: begin
                if(count == phase_cycles-1) begin
                    if(index == 0) begin
                        index <= 3'd7;
                        state <= ACK_SETUP;
                    end
                    else begin
                        index <= index - 1;
                        shift_reg <= shift_reg << 1;
                        state <= BIT_SETUP;
                    end
                    count <= 0;
                end
                else count <= count + 1;
            end
            ACK_SETUP: begin
                sda_low <= 1'b0;

                if(count == phase_cycles-1) begin
                    count <= 0;
                    scl_low <= 1'b0;
                    state <= ACK_HIGH;
                end
                else count <= count + 1;
            end
            ACK_HIGH: begin
                if(scl_bus) begin
                    if(count == phase_cycles-1) begin
                        target_acked <= !sda_bus;
                        count <= 0;
                        scl_low <= 1'b1;
                        state <= ACK_FALL;
                    end
                    else count <= count + 1;
                end
            end
            ACK_FALL: begin
                if(count == phase_cycles-1) begin
                    count <= 0;
                    if(!target_acked) begin
                        nack_detected <= 1'b1;
                        state <= STOP_LOW;
                    end
                    else if(sending_data_byte) begin
                        state <= STOP_LOW;
                    end
                    else if(rw_latched) begin
                        sda_low <= 1'b0;
                        state <= RX_SETUP;
                    end
                    else begin
                        sending_data_byte <= 1'b1;
                        shift_reg <= wr_data_latched;
                        state <= BIT_SETUP;
                    end
                end
                else count <= count + 1;
            end
            STOP_LOW: begin
                sda_low <= 1'b1;
                if(count==phase_cycles-1) begin
                    count <= 0;
                    scl_low <= 1'b0;
                    state <= STOP_HIGH;
                end
                else count <= count+1;
            end
            STOP_HIGH: begin
                if(scl_bus) begin
                    if(count==phase_cycles-1) begin
                        count <= 0;
                        sda_low <= 1'b0;
                        state <= STOP_RELEASE;
                    end
                    else count <= count+1;
                end
            end
            STOP_RELEASE: begin
                if(count==phase_cycles-1) begin
                    count <= 0;
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= IDLE;
                end
                else count <= count+1;
            end
            RX_SETUP: begin
                if(count==phase_cycles-1) begin
                    count <= 0;
                    scl_low <= 1'b0;
                    state <= RX_HIGH;
                end
                else count <= count+1;
            end
            RX_HIGH: begin
                if(scl_bus) begin
                   if(count==phase_cycles-1)begin
                       count<=0;
                       data_rcvd[index] <= sda_bus;
                       scl_low <= 1'b1;
                       state <= RX_FALL;
                   end 
                   else count <= count+1;
                end
            end
            RX_FALL: begin
                if(count==phase_cycles-1) begin
                    count<=0;
                    if(!index) begin
                        state <= RX_NACK_SETUP;
                    end
                    else begin
                        index <= index-1;
                        state <= RX_SETUP;
                    end
                end
                else count <= count+1;
            end
            RX_NACK_SETUP: begin
                if(count==phase_cycles-1) begin
                    count <=0;
                    scl_low <= 1'b0;
                    state <= RX_NACK_HIGH;
                end
                else count <= count+1;
            end
            RX_NACK_HIGH: begin
                if(scl_bus) begin
                    if(count==phase_cycles-1) begin
                        count <= 0;
                        scl_low <= 1'b1;
                        state <= STOP_LOW;
                    end
                    else count <= count+1;
                end
            end
            default: state <= IDLE;
        endcase
    end
end
endmodule