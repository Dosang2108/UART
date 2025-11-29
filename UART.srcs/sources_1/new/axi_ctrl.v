module axi4_ctrl #(
    parameter AXI4_CTRL_CONF     = 1,    // CONF_REG: On
    parameter AXI4_CTRL_STAT     = 0,    // STAT_REG: Off  
    parameter AXI4_CTRL_MEM      = 0,    // MEM:      Off
    parameter AXI4_CTRL_WR_ST    = 1,    // TX_FIFO:  On
    parameter AXI4_CTRL_RD_ST    = 1,    // RX_FIFO:  On
    parameter CONF_BASE_ADDR     = 32'h2000_0000,
    parameter CONF_OFFSET        = 32'h01,
    parameter CONF_REG_NUM       = 2,
    parameter ST_WR_BASE_ADDR    = 32'h2000_0010,
    parameter ST_WR_OFFSET       = 32'h01,
    parameter ST_WR_FIFO_NUM     = 1,
    parameter ST_WR_FIFO_DEPTH   = 4,
    parameter ST_RD_BASE_ADDR    = 32'h2000_0020,
    parameter ST_RD_OFFSET       = 32'h01,
    parameter ST_RD_FIFO_NUM     = 1,
    parameter ST_RD_FIFO_DEPTH   = 4,
    parameter DATA_W             = 8,
    parameter ADDR_W             = 32,
    parameter MST_ID_W           = 5,
    parameter TRANS_DATA_LEN_W   = 8,
    parameter TRANS_DATA_SIZE_W  = 3,
    parameter TRANS_RESP_W       = 2
) (
    input                           clk,
    input                           rst_n,
    // AXI4 Slave Interface
    input   [MST_ID_W-1:0]          m_awid_i,
    input   [ADDR_W-1:0]            m_awaddr_i,
    input   [1:0]                   m_awburst_i,
    input   [TRANS_DATA_LEN_W-1:0]  m_awlen_i,
    input                           m_awvalid_i,
    input   [DATA_W-1:0]            m_wdata_i,
    input                           m_wlast_i,
    input                           m_wvalid_i,
    input                           m_bready_i,
    input   [MST_ID_W-1:0]          m_arid_i,
    input   [ADDR_W-1:0]            m_araddr_i,
    input   [1:0]                   m_arburst_i,
    input   [TRANS_DATA_LEN_W-1:0]  m_arlen_i,
    input                           m_arvalid_i,
    input                           m_rready_i,
    // External Interfaces
    input   [DATA_W-1:0]            stat_reg_i,
    input                           mem_wr_rdy_i,
    input   [DATA_W-1:0]            mem_rd_data_i,
    input                           mem_rd_rdy_i,
    input   [ST_WR_FIFO_NUM-1:0]    wr_st_rd_vld_i,
    input   [DATA_W-1:0]            rd_st_wr_data_i,
    input   [ST_RD_FIFO_NUM-1:0]    rd_st_wr_vld_i,
    // AXI4 Slave Outputs
    output                          m_awready_o,
    output                          m_wready_o,
    output  [MST_ID_W-1:0]          m_bid_o,
    output  [TRANS_RESP_W-1:0]      m_bresp_o,
    output                          m_bvalid_o,
    output                          m_arready_o,
    output  [MST_ID_W-1:0]          m_rid_o,
    output  [DATA_W-1:0]            m_rdata_o,
    output  [TRANS_RESP_W-1:0]      m_rresp_o,
    output                          m_rlast_o,
    output                          m_rvalid_o,
    // Configuration & Memory Interfaces
    output  [DATA_W*CONF_REG_NUM-1:0]   conf_reg_o,
    output  [DATA_W-1:0]            mem_wr_data_o,
    output  [ADDR_W-1:0]            mem_wr_addr_o,
    output                          mem_wr_vld_o,
    output  [ADDR_W-1:0]            mem_rd_addr_o,
    output                          mem_rd_vld_o,
    // Stream Interfaces
    output  [DATA_W-1:0]            wr_st_rd_data_o,
    output  [ST_WR_FIFO_NUM-1:0]    wr_st_rd_rdy_o,
    output  [ST_RD_FIFO_NUM-1:0]    rd_st_wr_rdy_o
);

    // =========================================================================
    // Internal Signals
    // =========================================================================
    
    // AXI FSM States
    localparam [2:0] IDLE        = 3'b000;
    localparam [2:0] WRITE_ADDR  = 3'b001;
    localparam [2:0] WRITE_DATA  = 3'b010;
    localparam [2:0] WRITE_RESP  = 3'b011;
    localparam [2:0] READ_ADDR   = 3'b100;
    localparam [2:0] READ_DATA   = 3'b101;
    
    reg [2:0] current_state, next_state;
    
    // Address/Data Latches
    reg [MST_ID_W-1:0] awid_latch;
    reg [ADDR_W-1:0]   awaddr_latch;
    reg [MST_ID_W-1:0] arid_latch;
    reg [ADDR_W-1:0]   araddr_latch;

    // Decode using *latched* addresses
    wire wr_conf_sel;
    wire wr_st_sel;
    wire rd_conf_sel;
    wire rd_st_sel;

    assign wr_conf_sel = (awaddr_latch >= CONF_BASE_ADDR) &&
                         (awaddr_latch <  CONF_BASE_ADDR + CONF_OFFSET * CONF_REG_NUM);

    assign wr_st_sel   = (awaddr_latch >= ST_WR_BASE_ADDR) &&
                         (awaddr_latch <  ST_WR_BASE_ADDR + ST_WR_OFFSET * ST_WR_FIFO_NUM);

    assign rd_conf_sel = (araddr_latch >= CONF_BASE_ADDR) &&
                         (araddr_latch <  CONF_BASE_ADDR + CONF_OFFSET * CONF_REG_NUM);

    assign rd_st_sel   = (araddr_latch >= ST_RD_BASE_ADDR) &&
                         (araddr_latch <  ST_RD_BASE_ADDR + ST_RD_OFFSET * ST_RD_FIFO_NUM);

    // Configuration Registers (internal wires)
    wire [DATA_W*CONF_REG_NUM-1:0] conf_reg_int;
    wire [DATA_W-1:0]              conf_rd_data_int;
    
    // Stream FIFOs
    reg [DATA_W-1:0] wr_st_data;
    reg              wr_st_valid;
    reg              wr_st_ready;
    
    wire [DATA_W-1:0] rd_st_data;
    wire              rd_st_valid;
    reg               rd_st_ready;
    
    // AXI Control Signals
    reg awready;
    reg wready;
    reg bid_valid;
    reg arready;
    reg rvalid;
    reg rlast;
    
    // Response signals
    reg [TRANS_RESP_W-1:0] bresp;
    reg [TRANS_RESP_W-1:0] rresp;

    // Read data registers
    reg [DATA_W-1:0] read_data;
    reg              read_data_valid;

    // =========================================================================
    // AXI Finite State Machine
    // =========================================================================
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
        end
    end
    
    always @(*) begin
        next_state = current_state;
        
        case (current_state)
            IDLE: begin
                if (m_awvalid_i) begin
                    next_state = WRITE_ADDR;
                end else if (m_arvalid_i) begin
                    next_state = READ_ADDR;
                end
            end
            
            WRITE_ADDR: begin
                if (awready && m_awvalid_i) begin
                    next_state = WRITE_DATA;
                end
            end
            
            WRITE_DATA: begin
                if (wready && m_wvalid_i && m_wlast_i) begin
                    next_state = WRITE_RESP;
                end
            end
            
            WRITE_RESP: begin
                if (m_bvalid_o && m_bready_i) begin
                    next_state = IDLE;
                end
            end
            
            READ_ADDR: begin
                if (arready && m_arvalid_i) begin
                    next_state = READ_DATA;
                end
            end
            
            READ_DATA: begin
                if (m_rvalid_o && m_rready_i && m_rlast_o) begin
                    next_state = IDLE;
                end
            end
            
            default: next_state = IDLE;
        endcase
    end
    
    // =========================================================================
    // Write Address Channel
    // =========================================================================
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            awready      <= 1'b0;
            awid_latch   <= {MST_ID_W{1'b0}};
            awaddr_latch <= {ADDR_W{1'b0}};
        end else begin
            case (current_state)
                IDLE: begin
                    awready <= 1'b1;
                    if (m_awvalid_i) begin
                        awid_latch   <= m_awid_i;
                        awaddr_latch <= m_awaddr_i;
                    end
                end
                
                WRITE_ADDR: begin
                    if (awready && m_awvalid_i) begin
                        awready <= 1'b0;
                    end
                end
                
                default: awready <= 1'b0;
            endcase
        end
    end
    
    assign m_awready_o = awready;
    
    // =========================================================================
    // Write Data Channel  
    // =========================================================================
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wready      <= 1'b0;
            wr_st_data  <= {DATA_W{1'b0}};
            wr_st_valid <= 1'b0;
        end else begin
            case (current_state)
                WRITE_DATA: begin
                    // Ở đây vẫn đơn giản: luôn sẵn sàng nhận W
                    // (nếu muốn back-pressure theo wr_st_ready/mem_wr_rdy_i
                    // có thể AND thêm vào).
                    wready <= 1'b1;
                    
                    if (m_wvalid_i && wready) begin
                        // Lưu write data để đẩy ra stream / mem
                        wr_st_data  <= m_wdata_i;
                        wr_st_valid <= 1'b1;
                        
                        if (m_wlast_i) begin
                            wready <= 1'b0;
                        end
                    end else begin
                        wr_st_valid <= 1'b0;
                    end
                end
                
                default: begin
                    wready      <= 1'b0;
                    wr_st_valid <= 1'b0;
                end
            endcase
        end
    end
    
    assign m_wready_o = wready;
    
    // =========================================================================
    // Write Response Channel
    // =========================================================================
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bid_valid <= 1'b0;
            bresp     <= {TRANS_RESP_W{1'b0}};
        end else begin
            case (current_state)
                WRITE_RESP: begin
                    bid_valid <= 1'b1;
                    bresp     <= 2'b00; // OKAY response (chưa decode lỗi write)
                    
                    if (m_bvalid_o && m_bready_i) begin
                        bid_valid <= 1'b0;
                    end
                end
                
                default: bid_valid <= 1'b0;
            endcase
        end
    end
    
    assign m_bid_o    = awid_latch;
    assign m_bresp_o  = bresp;
    assign m_bvalid_o = bid_valid;
    
    // =========================================================================
    // Read Address Channel
    // =========================================================================
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arready      <= 1'b0;
            arid_latch   <= {MST_ID_W{1'b0}};
            araddr_latch <= {ADDR_W{1'b0}};
        end else begin
            case (current_state)
                IDLE: begin
                    arready <= 1'b1;
                    if (m_arvalid_i) begin
                        arid_latch   <= m_arid_i;
                        araddr_latch <= m_araddr_i;
                    end
                end
                
                READ_ADDR: begin
                    if (arready && m_arvalid_i) begin
                        arready <= 1'b0;
                    end
                end
                
                default: arready <= 1'b0;
            endcase
        end
    end
    
    assign m_arready_o = arready;
    
    // =========================================================================
    // Read Data Channel
    // =========================================================================
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rvalid          <= 1'b0;
            rlast           <= 1'b0;
            read_data       <= {DATA_W{1'b0}};
            read_data_valid <= 1'b0;
            rresp           <= {TRANS_RESP_W{1'b0}};
        end else begin
            case (current_state)
                READ_DATA: begin
                    // Defaults
                    read_data       <= {DATA_W{1'b0}};
                    read_data_valid <= 1'b0;
                    rresp           <= 2'b00; // OKAY
                    
                    // Configuration register read
                    if (AXI4_CTRL_CONF && rd_conf_sel) begin
                        read_data       <= conf_rd_data_int;
                        read_data_valid <= 1'b1; // thanh ghi luôn sẵn
                    end
                    // Read stream (RX FIFO)
                    else if (AXI4_CTRL_RD_ST && rd_st_sel) begin
                        read_data       <= rd_st_data;
                        read_data_valid <= rd_st_valid; // chờ FIFO có data
                    end
                    // Memory read
                    else if (AXI4_CTRL_MEM) begin
                        read_data       <= mem_rd_data_i;
                        read_data_valid <= mem_rd_rdy_i; // chờ MEM báo data ready
                    end
                    // Status register read (fallback)
                    else if (AXI4_CTRL_STAT) begin
                        read_data       <= stat_reg_i;
                        read_data_valid <= 1'b1;
                    end
                    // Invalid address -> SLVERR nhưng vẫn trả 1 beat
                    else begin
                        read_data       <= {DATA_W{1'b0}};
                        read_data_valid <= 1'b1;
                        rresp           <= 2'b10; // SLVERR
                    end
                    
                    // Do non-blocking nên rvalid = read_data_valid (cycle trước)
                    // => 1 chu kỳ để "prepare data", 1 chu kỳ để "handshake".
                    rvalid <= read_data_valid;
                    rlast  <= read_data_valid; // single transfer
                end
                
                default: begin
                    rvalid          <= 1'b0;
                    rlast           <= 1'b0;
                    read_data_valid <= 1'b0;
                    rresp           <= 2'b00; // OKAY
                end
            endcase
        end
    end
    
    assign m_rid_o   = arid_latch;
    assign m_rdata_o = read_data;
    assign m_rresp_o = rresp;
    assign m_rlast_o = rlast;
    assign m_rvalid_o= rvalid;
    
    // =========================================================================
    // Configuration Register Handling
    // =========================================================================
    
    generate
        if (AXI4_CTRL_CONF) begin : CONF_REG_GEN
            reg [DATA_W*CONF_REG_NUM-1:0] conf_reg_r;
            reg [DATA_W-1:0]              conf_rd_data_r;
            integer wr_index;
            integer rd_index;
            
            // Ghi thanh ghi cấu hình
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    conf_reg_r <= {DATA_W*CONF_REG_NUM{1'b0}};
                end else begin
                    if (current_state == WRITE_DATA && m_wvalid_i && wr_conf_sel) begin
                        wr_index = (awaddr_latch - CONF_BASE_ADDR) / CONF_OFFSET;
                        if (wr_index < CONF_REG_NUM) begin
                            conf_reg_r[wr_index*DATA_W +: DATA_W] <= m_wdata_i;
                        end
                    end
                end
            end
            
            // Đọc thanh ghi cấu hình
            always @(*) begin
                conf_rd_data_r = {DATA_W{1'b0}};
                if (rd_conf_sel) begin
                    rd_index = (araddr_latch - CONF_BASE_ADDR) / CONF_OFFSET;
                    if (rd_index < CONF_REG_NUM) begin
                        conf_rd_data_r = conf_reg_r[rd_index*DATA_W +: DATA_W];
                    end
                end
            end

            assign conf_reg_int     = conf_reg_r;
            assign conf_rd_data_int = conf_rd_data_r;
        end else begin : NO_CONF_REG
            assign conf_reg_int     = {DATA_W*CONF_REG_NUM{1'b0}};
            assign conf_rd_data_int = {DATA_W{1'b0}};
        end
    endgenerate
    
    assign conf_reg_o = conf_reg_int;
    
    // =========================================================================
    // Write Stream (TX FIFO) Interface
    // =========================================================================
    
    generate
        if (AXI4_CTRL_WR_ST) begin : WR_STREAM_GEN
            assign wr_st_rd_data_o = wr_st_data;
            // Ở đây vẫn giữ nguyên handshake "đơn giản":
            // module này khi có wr_st_valid thì báo ready cho tất cả FIFO.
            assign wr_st_rd_rdy_o  = {ST_WR_FIFO_NUM{wr_st_valid}};
            
            // wr_st_ready có thể dùng nếu bạn muốn chặn WREADY khi FIFO không nhận được
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    wr_st_ready <= 1'b0;
                end else begin
                    // Ít nhất 1 FIFO báo "ready" (ý nghĩa wr_st_rd_vld_i tùy design FIFO)
                    wr_st_ready <= |wr_st_rd_vld_i;
                end
            end
        end else begin : NO_WR_STREAM
            assign wr_st_rd_data_o = {DATA_W{1'b0}};
            assign wr_st_rd_rdy_o  = {ST_WR_FIFO_NUM{1'b0}};
        end
    endgenerate
    
    // =========================================================================
    // Read Stream (RX FIFO) Interface  
    // =========================================================================
    
    generate
        if (AXI4_CTRL_RD_ST) begin : RD_STREAM_GEN
            assign rd_st_data  = rd_st_wr_data_i;
            assign rd_st_valid = |rd_st_wr_vld_i; // At least one FIFO has data
            assign rd_st_wr_rdy_o = {ST_RD_FIFO_NUM{rd_st_ready}};
            
            // Ready khi đang READ_DATA và address thuộc vùng RD stream
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    rd_st_ready <= 1'b0;
                end else begin
                    rd_st_ready <= (current_state == READ_DATA) && rd_st_sel;
                end
            end
        end else begin : NO_RD_STREAM
            assign rd_st_data      = {DATA_W{1'b0}};
            assign rd_st_valid     = 1'b0;
            assign rd_st_wr_rdy_o  = {ST_RD_FIFO_NUM{1'b0}};
        end
    endgenerate
    
    // =========================================================================
    // Memory Interface (Optional)
    // =========================================================================
    
    generate
        if (AXI4_CTRL_MEM) begin : MEM_GEN
            // Memory write interface
            assign mem_wr_data_o = m_wdata_i;
            assign mem_wr_addr_o = awaddr_latch;
            assign mem_wr_vld_o  = (current_state == WRITE_DATA) && m_wvalid_i;
            
            // Memory read interface  
            assign mem_rd_addr_o = araddr_latch;
            assign mem_rd_vld_o  = (current_state == READ_ADDR) && m_arvalid_i;
        end else begin : NO_MEM
            assign mem_wr_data_o = {DATA_W{1'b0}};
            assign mem_wr_addr_o = {ADDR_W{1'b0}};
            assign mem_wr_vld_o  = 1'b0;
            assign mem_rd_addr_o = {ADDR_W{1'b0}};
            assign mem_rd_vld_o  = 1'b0;
        end
    endgenerate

endmodule
