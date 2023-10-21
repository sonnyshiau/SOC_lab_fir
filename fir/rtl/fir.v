// Company: NTHU EE
// Engineer: 蕭翔
// Create Date: 2023/10/19
`timescale 1ns / 1ps
module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    //Read address channel(RA)
    output  wire                     arready,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    input   wire                     arvalid, 
    //Read data channel(RD)
    output  reg                      rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,
    input   wire                     rready,
    //Write address channel(WA)
    output  wire                     awready,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     awvalid,
    //Write data channel(WD)
    output  wire                     wready,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,


    //streaming
    output  wire                     ss_tready,
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    input   wire                     sm_tready, 

    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

//------------------FSM----------------------//
parameter                   IDLE = 2'd0;
parameter                   TAP2BRAM = 2'd1;
parameter                   COMPUTE = 2'd2;
parameter                   DONE = 2'd3;
reg     [1:0]               next_state;
reg     [1:0]               STATE;

//------------------AXI LITE------------------//
reg                         tap_wdata_wen;  
reg [1:0]                   tap_rdata_cnt;
reg [5:0]                   tap_A_cnt;                                         
reg                         awready_reg;    
reg                         wready_reg;

reg [(pADDR_WIDTH-1):0]     tap_A_reg;
reg [(pDATA_WIDTH-1):0]     tap_Di_reg;
reg [3:0]                   tap_WE_reg;

reg                         arready_reg;
reg                         rvalid_reg;
//------------------AXI Streaming--------------//
reg    [(pDATA_WIDTH-1):0]  data_Di_reg;
reg    [(pADDR_WIDTH-1):0]  data_A_reg;
reg    [3:0]                data_WE_reg;

reg    [(pADDR_WIDTH-1):0]  cal_cnt;
reg    [3:0]                fifo_cnt_ptr,fifo_cnt_ptr_next;
reg    [9:0]                gold_num,gold_num_next;

reg    [(pDATA_WIDTH-1):0]  coef;
reg    [(pDATA_WIDTH-1):0]  data_Xn;
reg    [(pDATA_WIDTH-1):0]  mult_result;
reg    [(pDATA_WIDTH-1):0]  data_Yn;
wire                        one_cycle_done;
reg ap_done;

always@* ap_done = (STATE == DONE && rvalid) ? 1'b1 : 1'b0;

//fsm
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) STATE <= IDLE;
    else STATE <= next_state;
end

always @(*) begin
    case(STATE)
        IDLE:     next_state = (wdata == 32'd600) ? TAP2BRAM : IDLE;
        TAP2BRAM: next_state = (wvalid && wdata == 32'd1) ? COMPUTE : TAP2BRAM;
        COMPUTE:  next_state = (gold_num == 10'd600 && sm_tready && sm_tvalid) ? DONE : COMPUTE;
        DONE:     next_state = (ap_done) ? IDLE : DONE;
        default:  next_state = IDLE;  
    endcase
end

//tap bram
assign tap_EN = 1'd1;
assign tap_A = tap_A_reg;
assign tap_Di = tap_Di_reg;
assign tap_WE = tap_WE_reg;

//awvalid form tb to control tap_wdata_wen
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) tap_wdata_wen <= 1'b0;
    else if(awaddr != 12'd0) begin
        tap_wdata_wen <= 1'd0;
        if (awvalid)begin
            case(tap_wdata_wen)
            1'b0:tap_wdata_wen <= 1'd1;
            1'b1:tap_wdata_wen <= 1'd0;
            default:tap_wdata_wen <= 1'd0;
            endcase
        end
    end
    else tap_wdata_wen <= 1'd0;
end
//write hs control
assign awready = (tap_wdata_wen)? 1:0;
assign wready =  (tap_wdata_wen)? 1:0;
//read hs control
assign arready = (tap_rdata_cnt == 2'd1)? 1:0; 

//rvalid = arready delay one clk
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) rvalid <= 1'd0;
    else rvalid <= arready;
end

//tap_Di
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) tap_Di_reg <= 32'b0;
    else tap_Di_reg <= (wvalid && wready) ? wdata : 32'b0;
end
//tap_we
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) tap_WE_reg <= 4'd0;
    else tap_WE_reg <= ((wvalid && wready) && (awaddr != 12'd0)) ? 4'b1111 : 4'd0;
end
//tap_A_cnt control
always @(posedge axis_clk or negedge axis_rst_n) begin
    if (~axis_rst_n) begin
        tap_A_cnt <= 6'd0;
    end else begin
        case (STATE)
            TAP2BRAM: tap_A_cnt <= 6'd0;
            COMPUTE: begin
                if (data_WE == 4'b1111 || ss_tready) tap_A_cnt <= 6'd0;
                else tap_A_cnt <= tap_A_cnt + 1'd1;
            end
            default: tap_A_cnt <= tap_A_cnt;  
        endcase
    end
end
//tap_A control
always @(posedge axis_clk or negedge axis_rst_n) begin
    if (~axis_rst_n) begin
        tap_A_reg <= 12'd0;
    end else begin
        case (STATE)
            TAP2BRAM: begin
                if (awvalid && awready) begin
                    if (awaddr >= 12'd32) tap_A_reg <= awaddr - 12'd32;
                    else if (awaddr == 12'd16) tap_A_reg <= awaddr;
                    else tap_A_reg <= 12'd0;
                end 
                else if (arvalid) begin
                    if (araddr >= 12'd32) tap_A_reg <= araddr - 12'd32;
                    else if (araddr == 12'd0) tap_A_reg <= 12'd0;
                    else tap_A_reg <= tap_A_reg;
                end
                else tap_A_reg <= tap_A_reg;
            end
            COMPUTE: begin
                if (tap_A_cnt < 6'd11) tap_A_reg <= tap_A_cnt << 2;  //tap_A_cnt * 4
                else tap_A_reg <= tap_A_reg;
            end
            default: tap_A_reg <= tap_A_reg;
        endcase
    end
end



//axi-lite
assign rdata = (rvalid && (STATE == TAP2BRAM || STATE == COMPUTE))? tap_Do: (rvalid && STATE == DONE)? 32'd2:32'd4; 

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (~axis_rst_n) begin
        tap_rdata_cnt <= 2'b00;
    end else begin
        if (arvalid) begin
            case (tap_rdata_cnt)
                2'b00: tap_rdata_cnt <= 2'd1;
                2'b01: tap_rdata_cnt <= 2'd2;
                2'b10: tap_rdata_cnt <= 2'd0;
                default: tap_rdata_cnt <= tap_rdata_cnt;
            endcase
        end
        else tap_rdata_cnt <= tap_rdata_cnt;
    end
end


//data bram
assign data_Di = data_Di_reg;
assign data_A  = data_A_reg;
assign data_EN = 1'b1;
assign data_WE = data_WE_reg;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (~axis_rst_n) begin
        data_WE_reg <= 4'd0;
    end else begin
        case(STATE)
        TAP2BRAM: data_WE_reg <= (wvalid && wready && awaddr > 12'h000) ? 4'b1111 : 4'd0;
        COMPUTE:  data_WE_reg <= (ss_tready) ? 4'b1111 : 4'd0;
        default:  data_WE_reg <= 4'd0;
        endcase
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) data_Di_reg <= 32'b0;
    else begin
        case(STATE)
        TAP2BRAM:   data_Di_reg <= 32'b0;
        COMPUTE:    data_Di_reg <= ss_tdata;
        default:    data_Di_reg <= data_Di_reg;
        endcase
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        data_A_reg <= 12'h000;
    end else begin
        case (STATE)
        TAP2BRAM: begin
            if (awvalid && awready) data_A_reg <= (awaddr == 12'h010) ? 12'h000 : awaddr - 12'h020; 
            else if (arvalid) data_A_reg <= araddr - 12'h020;
            else data_A_reg <= data_A_reg;
        end
        COMPUTE: begin
            if (ss_tready) data_A_reg <= (fifo_cnt_ptr == 12'd11) ? 12'h000 : fifo_cnt_ptr << 2;
            else if (data_A_reg == 12'h000) data_A_reg <= 12'h028;
            else data_A_reg <= data_A_reg - 12'h004;
        end
        default: data_A_reg <= data_A_reg; 
        endcase
    end
end

//-----AXI stream------//
assign ss_tready = (STATE == COMPUTE && cal_cnt == 0)? 1:0;
assign sm_tvalid = (gold_num != 9'd1)? one_cycle_done:0;
assign sm_tlast =  (gold_num == 600 && one_cycle_done)? 1:0;

// Compute counter logic
always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) 
        cal_cnt <= 4'd0;
    else case(STATE)
        COMPUTE: begin
        cal_cnt <= cal_cnt;
        if (cal_cnt <= 4'd10) 
            cal_cnt <= cal_cnt + 1'b1;
        else if (cal_cnt == 4'd11) 
            cal_cnt <= 0;
        end
        default: cal_cnt <= cal_cnt;
    endcase
end

// Pattern cycle logic
always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) fifo_cnt_ptr <= 4'd0;
    else fifo_cnt_ptr <= fifo_cnt_ptr_next;
end

always@*begin
    case(STATE)
        COMPUTE: begin
        fifo_cnt_ptr_next = fifo_cnt_ptr;
        if (ss_tready && fifo_cnt_ptr < 4'd11) 
            fifo_cnt_ptr_next = fifo_cnt_ptr + 1;
        else if (fifo_cnt_ptr == 4'd11 && ss_tready) 
            fifo_cnt_ptr_next <= 1;
        end
        default: fifo_cnt_ptr_next = fifo_cnt_ptr;
    endcase
end

//gold_num
always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n)
        gold_num <= 0;
    else
        gold_num <= gold_num_next;
end
always@*begin
    if (STATE == COMPUTE && one_cycle_done)
        gold_num_next <= gold_num + 1;
    else
        gold_num_next <= gold_num;
end

//ss_tready delay 5 clk
assign one_cycle_done = (cal_cnt == 12'd4)? 1:0;

//tap_Do to coef_reg to calculate
// pipeline reg
reg [31:0] data_Do_pipeline [0:2];

//FIR calculate
always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        coef <= 0;
        data_Do_pipeline[0] <= 0;
        data_Do_pipeline[1] <= 0;
        data_Xn <= 0;
        mult_result <= 0;
    end else begin
        // tap_Do to coef_reg to calculate
        if (tap_A <= 12'h028) 
            coef <= tap_Do;
        else 
            coef <= coef;
            
        // data_Do delay 3clk to data_Xn
        if (data_WE == 4'b1111) 
            data_Do_pipeline[0] <= 0;
        else 
            data_Do_pipeline[0] <= data_Do;
        
        if (data_EN)
            data_Do_pipeline[1] <= data_Do_pipeline[0];
        else
            data_Do_pipeline[1] <= data_Do_pipeline[1];
            
        if (data_EN)
            data_Xn <= data_Do_pipeline[1];
        else
            data_Xn <= data_Xn;

        if (data_EN)
            mult_result <= data_Xn * coef;
        else
            mult_result <= mult_result;
    end
end


//data strm out sm_tdata
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) data_Yn <= 0;
    else if (data_EN && STATE == COMPUTE) begin
        if (~one_cycle_done) data_Yn <= data_Yn + mult_result;
        else data_Yn <= 0;
    end
    else data_Yn <= data_Yn;
end
assign sm_tdata = data_Yn;



endmodule