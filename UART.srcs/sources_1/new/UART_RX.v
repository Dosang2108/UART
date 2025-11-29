module uart_rx #(
    parameter DATA_WIDTH = 8,
    parameter TIMEOUT_CYCLES = 16
) (
    input                               clk,
    input                               RX,
    input                               baudrate_clk_en,
    output                              transaction_en,
    output [DATA_WIDTH - 1:0]           data_out_rx,
    output                              fifo_wr,
    output                              frame_error,
    output                              timeout_error,
    input                               rst_n
);
    
    // Fixed configuration
    localparam DATA_WIDTH_SELECT = 8;
    localparam PARITY_SELECT = 1;
    localparam STOP_BIT_SELECT = 1;
    
    // State definitions
    localparam IDLE_STATE   = 0;
    localparam RECV_STATE   = 1;
    
    localparam START_BIT_STATE  = 0;
    localparam DATA_STATE       = 1;
    localparam PARITY_STATE     = 2;
    localparam STOP_STATE       = 3;
    
    // Internal registers
    reg                             rx_state;
    reg [1:0]                       transaction_state;
    reg [DATA_WIDTH - 1:0]          rx_buffer;
    reg                             parity_buffer;
    reg                             fifo_wr_reg;
    reg                             frame_error_reg;
    
    reg [3:0]                       data_counter;
    reg                             stop_bit_counter;
    
    reg                             transaction_start_toggle;
    reg                             transaction_stop_toggle;
    
    // Timeout counter
    reg [7:0] timeout_counter;
    reg timeout_error_reg;
    
    // RX synchronization
    reg [2:0] rx_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync <= 3'b111;
        end else begin
            rx_sync <= {rx_sync[1:0], RX};
        end
    end
    
    wire rx_stable = rx_sync[2];
    
    // Edge detection for start bit
    reg rx_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_prev <= 1'b1;
        end else begin
            rx_prev <= rx_stable;
        end
    end
    
    wire start_bit_detected = (rx_prev == 1'b1) && (rx_stable == 1'b0);
    
    // Output assignments
    assign transaction_en = transaction_start_toggle ^ transaction_stop_toggle;
    assign data_out_rx = rx_buffer;
    assign fifo_wr = fifo_wr_reg;
    assign frame_error = frame_error_reg;
    assign timeout_error = timeout_error_reg;
    
    // Parity calculation
    wire calculated_parity = ^rx_buffer;
    wire parity_error = (PARITY_SELECT != 0) ? (calculated_parity != parity_buffer) : 1'b0;
    
    // Main RX state machine - SỬA: Reset logic tốt hơn
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            rx_state <= IDLE_STATE;
            transaction_start_toggle <= 0;
            fifo_wr_reg <= 0;
            frame_error_reg <= 0;
            timeout_error_reg <= 0;
            timeout_counter <= 0;
        end else begin
            // Default assignments
            fifo_wr_reg <= 0;
            frame_error_reg <= 0;
            timeout_error_reg <= 0;
            
            case(rx_state) 
                IDLE_STATE: begin
                    timeout_counter <= 0;
                    if(start_bit_detected) begin
                        rx_state <= RECV_STATE;
                        transaction_start_toggle <= ~transaction_start_toggle;
                        //$display("[RX] Start bit detected, beginning reception");
                    end
                end 
                RECV_STATE: begin
                    // Timeout counter
                    if (baudrate_clk_en) begin
                        if (timeout_counter < TIMEOUT_CYCLES) begin
                            timeout_counter <= timeout_counter + 1;
                        end else begin
                            // Timeout occurred
                            rx_state <= IDLE_STATE;
                            timeout_error_reg <= 1'b1;
                            transaction_stop_toggle <= transaction_start_toggle;
                            //$display("[RX] Timeout error detected");
                        end
                    end
                    
                    if(!transaction_en) begin
                        rx_state <= IDLE_STATE;
                        fifo_wr_reg <= 1'b1;
                        frame_error_reg <= parity_error;
                        //$display("[RX] Transaction complete, data=0x%02h, frame_error=%b", rx_buffer, parity_error);
                    end 
                end
                default: begin
                    rx_state <= IDLE_STATE;
                end
            endcase
        end
    end 
    
    // Transaction state machine - SỬA: Reset logic
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            transaction_state <= START_BIT_STATE;
            transaction_stop_toggle <= 0;
            parity_buffer <= 0;
            rx_buffer <= 0;
            data_counter <= 0;
            stop_bit_counter <= STOP_BIT_SELECT;
        end else begin
            if(rx_state == RECV_STATE) begin
                if (baudrate_clk_en) begin
                    case(transaction_state)
                        START_BIT_STATE: begin
                            if (rx_stable == 1'b0) begin
                                transaction_state <= DATA_STATE;
                                data_counter <= 0;
                                //$display("[RX] Start bit verified, moving to DATA state");
                            end else begin
                                // False start, abort
                                transaction_state <= START_BIT_STATE;
                                transaction_stop_toggle <= transaction_start_toggle;
                                //$display("[RX] False start bit detected, aborting");
                            end
                        end 
                        
                        DATA_STATE: begin
                            rx_buffer[data_counter] <= rx_stable;
                            //$display("[RX] Data bit %0d = %b, buffer=0x%02h", data_counter, rx_stable, rx_buffer);
                            
                            if (data_counter == (DATA_WIDTH - 1)) begin
                                if (PARITY_SELECT != 0) begin
                                    transaction_state <= PARITY_STATE;
                                    //$display("[RX] All data bits received, moving to PARITY state");
                                end else begin
                                    transaction_state <= STOP_STATE;
                                    stop_bit_counter <= STOP_BIT_SELECT;
                                    //$display("[RX] All data bits received (no parity), moving to STOP state");
                                end
                            end else begin
                                data_counter <= data_counter + 1;
                            end
                        end 
                        
                        PARITY_STATE: begin
                            parity_buffer <= rx_stable;
                            transaction_state <= STOP_STATE;
                            stop_bit_counter <= STOP_BIT_SELECT;
                            //$display("[RX] Parity bit = %b, expected = %b", rx_stable, calculated_parity);
                        end 
                        
                        STOP_STATE: begin
                            if (rx_stable == 1'b1) begin
                                if (stop_bit_counter == 0) begin
                                    transaction_state <= START_BIT_STATE;
                                    transaction_stop_toggle <= transaction_start_toggle;
                                    //$display("[RX] All stop bits received, transaction complete");
                                end else begin
                                    stop_bit_counter <= stop_bit_counter - 1;
                                    //$display("[RX] Stop bit %0d received", STOP_BIT_SELECT - stop_bit_counter + 1);
                                end
                            end else begin
                                transaction_state <= START_BIT_STATE;
                                transaction_stop_toggle <= transaction_start_toggle;
                                frame_error_reg <= 1'b1;
                                //$display("[RX] Invalid stop bit detected, frame error");
                            end
                        end
                        
                        default: transaction_state <= START_BIT_STATE;
                    endcase 
                end
            end else begin
                // Reset transaction state when not in RECV_STATE
                if (rx_state == IDLE_STATE) begin
                    transaction_state <= START_BIT_STATE;
                    transaction_stop_toggle <= transaction_start_toggle;
                    data_counter <= 0;
                    stop_bit_counter <= STOP_BIT_SELECT;
                end
            end
        end
    end

endmodule