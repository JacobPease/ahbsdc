//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2013-2022 Authors                              ////
////                                                              ////
//// Based on original work by                                    ////
////     Adam Edvardsson (adam.edvardsson@orsoc.se)               ////
////                                                              ////
////     Copyright (C) 2009 Authors                               ////
////                                                              ////
//// This source file may be used and distributed without         ////
//// restriction provided that this copyright statement is not    ////
//// removed from the file and that any derivative work contains  ////
//// the original copyright notice and the associated disclaimer. ////
////                                                              ////
//// This source file is free software; you can redistribute it   ////
//// and/or modify it under the terms of the GNU Lesser General   ////
//// Public License as published by the Free Software Foundation; ////
//// either version 2.1 of the License, or (at your option) any   ////
//// later version.                                               ////
////                                                              ////
//// This source is distributed in the hope that it will be       ////
//// useful, but WITHOUT ANY WARRANTY; without even the implied   ////
//// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ////
//// PURPOSE. See the GNU Lesser General Public License for more  ////
//// details.                                                     ////
////                                                              ////
//// You should have received a copy of the GNU Lesser General    ////
//// Public License along with this source; if not, download it   ////
//// from https://www.gnu.org/licenses/                           ////
////                                                              ////
//////////////////////////////////////////////////////////////////////

module sdc_ahb import cvw::*; #(
  parameter dma_addr_bits = 32,
  parameter fifo_addr_bits = 7,
  parameter sdio_card_detect_level = 1,
  parameter voltage_controll_reg = 3300,
  parameter capabilies_reg = 16'b0000_0000_0000_0011,
  parameter cvw_t P
)(
  // Reset is already synched. Kept commented just in case.
  // input logic                     async_resetn,
  input logic                  HCLK, HRESETn,

  // Slave Interface
  input logic                  HSELSDC,
  input logic [P.PA_BITS-1:0]  HADDR,
  input logic [P.AHBW-1:0]     HWDATA,
  input logic [P.XLEN/8-1:0]   HWSTRB,
  input logic                  HWRITE,
  input logic [2:0]            HSIZE,
  // input logic [2:0]           HBURST, // Not implemented for slave interface
  input logic [1:0]            HTRANS,
  output logic [P.AHBW-1:0]    HREADSDC,
  output logic                 HRESPSDC, HREADYSDC, // To bus

  // Master Interface
  output logic [P.PA_BITS-1:0] SDCHADDR,
  output logic [P.AHBW-1:0]    SDCHWDATA,
  output logic [P.XLEN/8-1:0]  SDCHWSTRB,
  output logic                 SDCHWRITE,
  output logic [2:0]           SDCHSIZE,
  output logic [2:0]           SDCHBURST,
  output logic [1:0]           SDCHTRANS,
  input logic [P.AHBW-1:0]     HRDATA,
  input logic                  HRESP,
  input logic                  HREADY, // From bus
  
  // SD Card Signals
  output logic                 sdio_clk,
  output logic                 sdio_reset,
  input logic                  sdio_cd,

  // DAT IOBUF Control
  output logic                 sd_dat_reg_t,
  output logic [3:0]           sd_dat_reg_o,
  input logic [3:0]            sd_dat_i,

  // CMD IOBUF Control                    
  output logic                 sd_cmd_reg_t,
  output logic                 sd_cmd_reg_o,
  input logic                  sd_cmd_i,

  // Interrupts
  output logic                 interrupt
);

  `include "sd_defines.h"

  logic                        reset;

  logic                        go_idle;
  logic                        cmd_start;
  logic [1:0]                  cmd_setting;
  logic                        cmd_start_tx;
  logic [39:0]                 cmd;
  logic [119:0]                cmd_response;
  logic                        cmd_crc_ok;
  logic                        cmd_index_ok;
  logic                        cmd_finish;

  logic                        d_write;
  logic                        d_read;
  logic [31:0]                 data_in_rx_fifo;
  logic                        en_tx_fifo;
  logic                        en_rx_fifo;
  logic                        sd_data_busy;
  logic                        data_busy;
  logic                        data_crc_ok;
  logic                        tx_fifo_re;
  logic                        rx_fifo_we;

  logic                        data_start_rx;
  logic                        data_start_tx;
  logic                        data_prepare_tx;
  logic                        cmd_int_rst;
  logic                        data_int_rst;
  logic                        ctrl_rst;

  // AHB accessible registers
  logic [31:0]                 argument_reg;
  logic [`CMD_REG_SIZE-1:0]    command_reg;
  logic [`CMD_TIMEOUT_W-1:0]   cmd_timeout_reg;
  logic [`DATA_TIMEOUT_W-1:0]  data_timeout_reg;
  logic [0:0]                  software_reset_reg;
  logic [31:0]                 response_0_reg;
  logic [31:0]                 response_1_reg;
  logic [31:0]                 response_2_reg;
  logic [31:0]                 response_3_reg;
  logic [`BLKSIZE_W-1:0]       block_size_reg;
  logic [1:0]                  controller_setting_reg;
  logic [`INT_CMD_SIZE-1:0]    cmd_int_status_reg;
  logic [`INT_DATA_SIZE-1:0]   data_int_status_reg;
  logic [`INT_DATA_SIZE-1:0]   data_int_status;
  logic [`INT_CMD_SIZE-1:0]    cmd_int_enable_reg;
  logic [`INT_DATA_SIZE-1:0]   data_int_enable_reg;
  logic [`BLKCNT_W-1:0]        block_count_reg;
  logic [dma_addr_bits-1:0]    dma_addr_reg;
  logic [7:0]                  clock_divider_reg = 124; // 400KHz


  logic [dma_addr_bits-1:0] m_axi_awaddr;
  logic [7:0]               m_axi_awlen;
  logic                     m_axi_awvalid;
  logic                     m_axi_awready;
  logic [31:0]             m_axi_wdata;
  logic                     m_axi_wlast;
  logic                     m_axi_wvalid;
  logic                     m_axi_wready;
  logic [1:0]               m_axi_bresp;
  logic                     m_axi_bvalid;
  logic                    m_axi_bready;
  logic [dma_addr_bits-1:0] m_axi_araddr;
  logic [7:0]               m_axi_arlen;
  logic                     m_axi_arvalid;
  logic                     m_axi_arready;
  logic [31:0]              m_axi_rdata;
  logic                     m_axi_rlast;
  logic [1:0]               m_axi_rresp;
  logic                     m_axi_rvalid;
  logic                    m_axi_rready;
  
  
// ------ Clocks and resets

/*(* ASYNC_REG="true" *)
logic  [2:0] reset_sync;
assign reset = reset_sync[2];

always @(posedge clock)
    reset_sync <= {reset_sync[1:0], !async_resetn};*/
  
  // Synced clock from bus
  assign reset = ~HRESETn;

  logic [7:0]                  clock_cnt;
  logic                        clock_state;
  logic                        clock_posedge;
  logic                        clock_data_in;
  logic                        fifo_almost_full;
  logic                        fifo_almost_empty;

  always @(posedge HCLK) begin
    if (reset) begin
      clock_posedge <= 0;
      clock_data_in <= 0;
      clock_state <= 0;
      clock_cnt <= 0;
    end else if (clock_cnt < clock_divider_reg) begin
      clock_posedge <= 0;
      clock_data_in <= 0;
      clock_cnt <= clock_cnt + 1;
    end else if (clock_cnt < 124 && data_busy && en_rx_fifo && fifo_almost_full) begin
      // Prevent Rx FIFO overflow
      clock_posedge <= 0;
      clock_data_in <= 0;
      clock_cnt <= clock_cnt + 1;
    end else if (clock_cnt < 124 && data_busy && en_tx_fifo && fifo_almost_empty) begin
      // Prevent Tx FIFO underflow
      clock_posedge <= 0;
      clock_data_in <= 0;
      clock_cnt <= clock_cnt + 1;
    end else begin
      clock_state <= !clock_state;
      clock_posedge <= !clock_state;
      if (clock_divider_reg == 0)
        clock_data_in <= !clock_state;
      else
        clock_data_in <= clock_state;
      clock_cnt <= 0;
    end
    sdio_clk <= sdio_reset || clock_state;

    if (reset) sdio_reset <= 0;
    else if (clock_posedge) sdio_reset <= controller_setting_reg[1];
  end

// ------ SD IO Buffers

  logic sd_cmd_o;
  logic sd_cmd_oe;
  logic [3:0] sd_dat_o;
  logic       sd_dat_oe;

  always @(negedge HCLK) begin
    // Output data delayed by 1/2 clock cycle (5ns) to ensure
    // required hold time: default speed - min 5ns, high speed - min 2ns (actual 5ns)
    if (sdio_reset) begin
      sd_cmd_reg_o <= 0;
      sd_dat_reg_o <= 0;
      sd_cmd_reg_t <= 0;
      sd_dat_reg_t <= 0;
    end else begin
      sd_cmd_reg_o <= sd_cmd_o;
      sd_dat_reg_o <= sd_dat_o;
      sd_cmd_reg_t <= !sd_cmd_oe;
      sd_dat_reg_t <= !(sd_dat_oe || (cmd_start_tx && (command_reg == 0)));
    end
  end

// ------ SD card detect

  logic [25:0] sd_detect_cnt;
  logic        sd_insert_int = sd_detect_cnt[25];
  logic        sd_remove_int = !sd_detect_cnt[25];
  logic        sd_insert_ie;
  logic        sd_remove_ie;

  always @(posedge HCLK) begin
    if (sdio_cd != sdio_card_detect_level) begin
      sd_detect_cnt <= 0;
    end else if (!sd_insert_int) begin
      sd_detect_cnt <= sd_detect_cnt + 1;
    end
  end

// ------ AXI Slave Interface

  logic [15:0] read_addr;
  logic [15:0] write_addr;
  logic [31:0] write_data;
  logic        rd_req;
  logic        wr_req;
  logic        initTrans;
  logic        memwrite;
  logic        memread;
  logic [31:0] HREADWORD;
  

  /*assign s_axi_arready = !rd_req && !s_axi_rvalid;
  assign s_axi_awready = !wr_req[0] && !s_axi_bvalid;
  assign s_axi_wready = !wr_req[1] && !s_axi_bvalid;*/

  assign initTrans = HREADY & HSELSDC & HTRANS[1];
  assign memwrite = initTrans & HWRITE;
  assign memread = initTrans & ~HWRITE;
  assign HREADYSDC = 1'b1;

  assign HREADSDC = {HREADWORD, HREADWORD};
  

  always @(posedge HCLK) begin
    if (reset) begin
      HREADWORD <= 0;
      //HREADYSDC <= 1;
      HRESPSDC <= 0;
      rd_req <= 0;
      wr_req <= 0;
      read_addr <= 0;
      write_addr <= 0;
      write_data <= 0;
      cmd_start <= 0;
      data_int_rst <= 0;
      cmd_int_rst <= 0;
      ctrl_rst <= 0;
      argument_reg <= 0;
      command_reg <= 0;
      cmd_timeout_reg <= 0;
      data_timeout_reg <= 0;
      block_size_reg <= `RESET_BLOCK_SIZE;
      controller_setting_reg <= 0;
      cmd_int_enable_reg <= 0;
      data_int_enable_reg <= 0;
      software_reset_reg <= 0;
      clock_divider_reg <= `RESET_CLOCK_DIV;
      block_count_reg <= 0;
      sd_insert_ie <= 0;
      sd_remove_ie <= 0;
      dma_addr_reg <= 0;
    end else begin
      if (clock_posedge) begin
        cmd_start <= 0;
        data_int_rst <= 0;
        cmd_int_rst <= 0;
        ctrl_rst <= software_reset_reg[0];
      end
      
      if (memread) begin
        case (HADDR[7:0])
          `argument     : HREADWORD <= #1 argument_reg;
          `command      : HREADWORD <= #1 command_reg;
          `resp0        : HREADWORD <= #1 response_0_reg;
          `resp1        : HREADWORD <= #1 response_1_reg;
          `resp2        : HREADWORD <= #1 response_2_reg;
          `resp3        : HREADWORD <= #1 response_3_reg;
          `controller   : HREADWORD <= #1 controller_setting_reg;
          `blksize      : HREADWORD <= #1 block_size_reg;
          `voltage      : HREADWORD <= #1 voltage_controll_reg;
          `capa         : HREADWORD <= #1 capabilies_reg | (dma_addr_bits << 8);
          `clock_d      : HREADWORD <= #1 clock_divider_reg;
          `reset        : HREADWORD <= #1 { cmd_start, data_int_rst, cmd_int_rst, ctrl_rst };
          `cmd_timeout  : HREADWORD <= #1 cmd_timeout_reg;
          `data_timeout : HREADWORD <= #1 data_timeout_reg;
          `cmd_isr      : HREADWORD <= #1 cmd_int_status_reg;
          `cmd_iser     : HREADWORD <= #1 cmd_int_enable_reg;
          `data_isr     : HREADWORD <= #1 data_int_status_reg;
          `data_iser    : HREADWORD <= #1 data_int_enable_reg;
          `blkcnt       : HREADWORD <= #1 block_count_reg;
          `card_detect  : HREADWORD <= #1 { sd_remove_int, sd_remove_ie, sd_insert_int, sd_insert_ie };
          `dst_src_addr : HREADWORD <= #1 dma_addr_reg[31:0];
          `dst_src_addr_high : if (dma_addr_bits > 32) HREADWORD <= #1 dma_addr_reg[dma_addr_bits-1:32];
        endcase
      end else HREADWORD <= 0;
      
      if (memwrite) begin
        write_addr <= HADDR;
        wr_req <= 1;
      end
      
      if (wr_req) begin
          case (write_addr[7:0])
            `argument     : begin argument_reg <= HWDATA; cmd_start <= 1; end
            `command      : command_reg <= HWDATA;
            `reset        : software_reset_reg <= HWDATA;
            `cmd_timeout  : cmd_timeout_reg <= HWDATA;
            `data_timeout : data_timeout_reg <= HWDATA;
            `blksize      : block_size_reg <= HWDATA;
            `controller   : controller_setting_reg <= HWDATA;
            `cmd_isr      : cmd_int_rst <= 1;
            `cmd_iser     : cmd_int_enable_reg <= HWDATA;
            `clock_d      : clock_divider_reg <= HWDATA;
            `data_isr     : data_int_rst <= 1;
            `data_iser    : data_int_enable_reg <= HWDATA;
            `blkcnt       : block_count_reg <= HWDATA;
            `card_detect  : begin sd_remove_ie <= HWDATA[2]; sd_insert_ie <= HWDATA[0]; end
            `dst_src_addr : dma_addr_reg[31:0] <= HWDATA;
            `dst_src_addr_high : if (dma_addr_bits > 32) dma_addr_reg[dma_addr_bits-1:32] <= HWDATA;
          endcase
        wr_req <= 0;
      end
    end
  end

// ------ Data FIFO

  logic [31:0] m_bus_dat_i; // Read data from bus
  logic [2:0] m_axi_bresp_cnt;
  
logic  [31:0] fifo_mem [(1<<fifo_addr_bits)-1:0];
logic  [fifo_addr_bits-1:0] fifo_inp_pos;
logic  [fifo_addr_bits-1:0] fifo_out_pos;
logic [fifo_addr_bits-1:0] fifo_inp_nxt = fifo_inp_pos + 1;
logic [fifo_addr_bits-1:0] fifo_out_nxt = fifo_out_pos + 1;
logic [fifo_addr_bits-1:0] fifo_data_len = fifo_inp_pos - fifo_out_pos;
logic [fifo_addr_bits-1:0] fifo_free_len = fifo_out_pos - fifo_inp_nxt; // The free space available in fifo
logic fifo_full = fifo_inp_nxt == fifo_out_pos;
logic fifo_empty = fifo_inp_pos == fifo_out_pos;
logic fifo_ready = fifo_data_len >= (1 << fifo_addr_bits) / 2;
logic [31:0] fifo_din = en_rx_fifo ? data_in_rx_fifo : m_bus_dat_i; // INAXI: m_bus_dat_i? What's that?
// OUTAXI: So if rx fifo is disabled, write enable for the fifo is only enabled when rready and rvalid
  // are both high? So only allow writes to the fifo if the bus is ready to accept them,

  // Something tells me this is the spot where it chooses to talk to the sdcard or bus
logic fifo_we = en_rx_fifo ? rx_fifo_we && clock_posedge : m_axi_rready && m_axi_rvalid;
logic fifo_re = en_rx_fifo ? m_axi_wready && m_axi_wvalid : tx_fifo_re && clock_posedge;
logic [31:0] fifo_dout;

assign fifo_almost_full = fifo_data_len > (1 << fifo_addr_bits) * 3 / 4;
assign fifo_almost_empty = fifo_free_len > (1 << fifo_addr_bits) * 3 / 4;

  // I have no idea what stb stands for, but it must be something like start buffer?
logic tx_stb = en_tx_fifo && fifo_free_len >= (1 << fifo_addr_bits) / 3;
logic rx_stb = en_rx_fifo && m_axi_bresp_cnt != 3'b111 && (fifo_data_len >= (1 << fifo_addr_bits) / 3 || (!fifo_empty && !data_busy));

// Based on what I can tell, this fifo doubles as both the
// transfer fifo and the receiver fifo depending on what state
// sd_data_master is in.
  always @(posedge HCLK)
    if (reset || ctrl_rst || !(en_rx_fifo || en_tx_fifo)) begin // If neither fifo is enabled don't use them?
      fifo_inp_pos <= 0;
      fifo_out_pos <= 0;
    end else begin
      if (fifo_we && !fifo_full) begin
        fifo_mem[fifo_inp_pos] <= fifo_din;
        fifo_inp_pos <= fifo_inp_nxt;
        if (fifo_empty) fifo_dout <= fifo_din;
      end
      if (fifo_re && !fifo_empty) begin
        if (fifo_we && !fifo_full && fifo_out_nxt == fifo_inp_pos) fifo_dout <= fifo_din;
        else fifo_dout <= fifo_mem[fifo_out_nxt];
        fifo_out_pos <= fifo_out_nxt;
      end
    end

// ------ AXI Master Interface

// AXI transaction (DDR access) is over 80 clock cycles
// Must use burst to achive required throughput

  logic m_axi_cyc; // Represents bus cycle
  // assigning m_axi_write as en_rx_fifo is interesting. This must be receiving from
  // the sdcard. This must mean that:
  // txfifo -> transferring from memory to sdcard
  // rxfifo -> receiving from sdcard and placing it into memory.
logic m_axi_write = en_rx_fifo; // Receiver fifo... receiving from SD card?
logic [7:0] m_axi_wcnt; // 
logic [dma_addr_bits-1:2] m_bus_adr_o; 
logic m_bus_error; // 

assign m_axi_bready = m_axi_bresp_cnt != 0; // OUTAXI
assign m_axi_rready = m_axi_cyc & !m_axi_write; // OUTAXI
// OUTAXI: It appears that m_bus_dat_i is simply and endian swapped rdata. Interesting. 
assign m_bus_dat_i = {m_axi_rdata[7:0],m_axi_rdata[15:8],m_axi_rdata[23:16],m_axi_rdata[31:24]};
assign m_axi_wdata = {fifo_dout[7:0],fifo_dout[15:8],fifo_dout[23:16],fifo_dout[31:24]};

// AXI burst cannot cross a 4KB boundary
logic [fifo_addr_bits-1:0] tx_burst_len;
logic [fifo_addr_bits-1:0] rx_burst_len;
assign tx_burst_len = m_bus_adr_o[11:2] + fifo_free_len >= m_bus_adr_o[11:2] ? fifo_free_len - 1 : ~m_bus_adr_o[fifo_addr_bits+1:2];
assign rx_burst_len = m_bus_adr_o[11:2] + fifo_data_len >= m_bus_adr_o[11:2] ? fifo_data_len - 1 : ~m_bus_adr_o[fifo_addr_bits+1:2];

assign data_int_status_reg = { data_int_status[`INT_DATA_SIZE-1:1],
    !en_rx_fifo && !en_tx_fifo && !m_axi_cyc && m_axi_bresp_cnt == 0 && data_int_status[0] };

  logic                    ahb_cyc;

  // constants
  assign SDCHWSTRB = '1;
  assign SDCHWRITE = en_rx_fifo;
  
  always @(posedge HCLK) begin
    // SDC Master Bus Control
    if (reset | ctrl_rst) begin
      SDCHADDR <= 0;
      SDCHBURST <= 0;
      SDCHSIZE <= 0;
      SDCHBURST <= 0;
      SDCHTRANS <= 0;
    end else if (ahb_cyc) begin
      
    end else if (tx_stb || rx_stb) begin
      ahb_cyc <= 1;
    end

    // m_bus_adr_o assignment. This helps axi burst stuff
    // This appears to be used to control rx_burst_len and tx_burst_len
    // Which in turn is used to control m_axi_arlen and m_axi_awlen
    if (reset | ctrl_rst) begin
        m_bus_adr_o <= 0;
    end else if ((m_axi_wready && m_axi_wvalid) || (m_axi_rready && m_axi_rvalid)) begin
        m_bus_adr_o <= m_bus_adr_o + 1;
    end else if (!m_axi_cyc && !en_rx_fifo && !en_tx_fifo) begin
        m_bus_adr_o <= dma_addr_reg[dma_addr_bits-1:2];
    end

    // m_axi_bresp_cnt assignment
    if (reset | ctrl_rst) begin
        m_axi_bresp_cnt <= 0;
    end else if ((m_axi_awvalid && m_axi_awready) && !(m_axi_bvalid && m_axi_bready)) begin
        m_axi_bresp_cnt <= m_axi_bresp_cnt + 1;
    end else if (!(m_axi_awvalid && m_axi_awready) && (m_axi_bvalid && m_axi_bready)) begin
        m_axi_bresp_cnt <= m_axi_bresp_cnt - 1;
    end

    // m_bus_error assignment
    if (reset | ctrl_rst | cmd_start) begin
        m_bus_error <= 0;
    end else if (m_axi_bvalid && m_axi_bready && m_axi_bresp) begin
        m_bus_error <= 1;
    end else if (m_axi_rvalid && m_axi_rready && m_axi_rresp) begin
        m_bus_error <= 1;
    end
    
    // This controls whether we use the receiver fifo or transfer fifo
    // i.e. Whether we are transferring from sd -> mem (rx) or mem -> sd (tx)
    if (reset | ctrl_rst) begin
      data_start_tx <= 0;
      data_start_rx <= 0;
      data_prepare_tx <= 0;
    end else if (clock_posedge) begin
      data_start_tx <= 0;
      data_start_rx <= 0;
      if (cmd_start) begin
        data_prepare_tx <= 0;
        if (command_reg[`CMD_WITH_DATA] == 2'b01) data_start_rx <= 1;
        else if (command_reg[`CMD_WITH_DATA] != 2'b00) data_prepare_tx <= 1;
      end else if (data_prepare_tx) begin
        if (cmd_int_status_reg[`INT_CMD_CC]) begin
          data_prepare_tx <= 0;
          data_start_tx <= 1;
        end else if (cmd_int_status_reg[`INT_CMD_EI]) begin
          data_prepare_tx <= 0;
        end
      end
    end
    
  end
  
  
/*always @(posedge HCLK) begin
    if (reset | ctrl_rst) begin
        m_axi_arvalid <= 0;
        m_axi_awvalid <= 0;
        m_axi_wvalid <= 0;
        m_axi_cyc <= 0;
    end else if (m_axi_cyc) begin // Handle the rest of the bus cycle
        if (m_axi_awvalid && m_axi_awready) begin
            m_axi_awvalid <= 0;
        end
        if (m_axi_arvalid && m_axi_arready) begin
            m_axi_arvalid <= 0;
        end
        if (m_axi_wvalid && m_axi_wready) begin
            if (m_axi_wlast) begin
                m_axi_wvalid <= 0;
                m_axi_cyc <= 0;
            end else begin
                m_axi_wlast <= m_axi_wcnt + 1 == m_axi_awlen; // taking advantage of burst here
                m_axi_wcnt <= m_axi_wcnt + 1;
            end
        end
        if (m_axi_rvalid && m_axi_rready && m_axi_rlast) begin
            m_axi_cyc <= 0;
        end
    end else if (tx_stb || rx_stb) begin
        m_axi_cyc <= 1; // begin bus cycle
        m_axi_wcnt <= 0;
        if (m_axi_write) begin // set write axi channel
            m_axi_awaddr <= { m_bus_adr_o, 2'b00 };
            m_axi_awlen <= rx_burst_len < 8'hff ? rx_burst_len : 8'hff;
            m_axi_wlast <= rx_burst_len == 0;
            m_axi_awvalid <= 1;
            m_axi_wvalid <= 1;
        end else begin // set read axi channel values
            m_axi_araddr <= { m_bus_adr_o, 2'b00 };
            m_axi_arlen <= tx_burst_len < 8'hff ? tx_burst_len : 8'hff;
            m_axi_arvalid <= 1;
        end
    end

    // m_bus_adr_o assignment
    if (reset | ctrl_rst) begin
        m_bus_adr_o <= 0;
    end else if ((m_axi_wready && m_axi_wvalid) || (m_axi_rready && m_axi_rvalid)) begin
        m_bus_adr_o <= m_bus_adr_o + 1;
    end else if (!m_axi_cyc && !en_rx_fifo && !en_tx_fifo) begin
        m_bus_adr_o <= dma_addr_reg[dma_addr_bits-1:2];
    end

    // m_axi_bresp_cnt assignment
    // This appears to be used to control rx_burst_len and tx_burst_len
    // Which in turn is used to control m_axi_arlen and m_axi_awlen
    if (reset | ctrl_rst) begin
        m_axi_bresp_cnt <= 0;
    end else if ((m_axi_awvalid && m_axi_awready) && !(m_axi_bvalid && m_axi_bready)) begin
        m_axi_bresp_cnt <= m_axi_bresp_cnt + 1;
    end else if (!(m_axi_awvalid && m_axi_awready) && (m_axi_bvalid && m_axi_bready)) begin
        m_axi_bresp_cnt <= m_axi_bresp_cnt - 1;
    end

    // m_bus_error assignment
    if (reset | ctrl_rst | cmd_start) begin
        m_bus_error <= 0;
    end else if (m_axi_bvalid && m_axi_bready && m_axi_bresp) begin
        m_bus_error <= 1;
    end else if (m_axi_rvalid && m_axi_rready && m_axi_rresp) begin
        m_bus_error <= 1;
    end

  // This part is used to start receiving or transferring depending on
  // what kind of command it is we are using. This part is essential.
  // The transfer fifo can be used either to write to the SD card or to
  // Main memory.
    if (reset | ctrl_rst) begin
        data_start_tx <= 0;
        data_start_rx <= 0;
        data_prepare_tx <= 0;
    end else if (clock_posedge) begin
        data_start_tx <= 0;
        data_start_rx <= 0;
        if (cmd_start) begin
            data_prepare_tx <= 0;
            if (command_reg[`CMD_WITH_DATA] == 2'b01) data_start_rx <= 1;
            else if (command_reg[`CMD_WITH_DATA] != 2'b00) data_prepare_tx <= 1;
        end else if (data_prepare_tx) begin
            if (cmd_int_status_reg[`INT_CMD_CC]) begin
                data_prepare_tx <= 0;
                data_start_tx <= 1;
            end else if (cmd_int_status_reg[`INT_CMD_EI]) begin
                data_prepare_tx <= 0;
            end
        end
    end
end*/

// ------ SD Card Interface

sd_cmd_master sd_cmd_master0(
    .clock            (HCLK),
    .clock_posedge    (clock_posedge),
    .reset            (reset | ctrl_rst),
    .start            (cmd_start),
    .int_status_rst   (cmd_int_rst),
    .setting          (cmd_setting),
    .start_xfr        (cmd_start_tx),
    .go_idle          (go_idle),
    .cmd              (cmd),
    .response         (cmd_response),
    .crc_error        (!cmd_crc_ok),
    .index_ok         (cmd_index_ok),
    .busy             (sd_data_busy),
    .finish           (cmd_finish),
    .argument         (argument_reg),
    .command          (command_reg),
    .timeout          (cmd_timeout_reg),
    .int_status       (cmd_int_status_reg),
    .response_0       (response_0_reg),
    .response_1       (response_1_reg),
    .response_2       (response_2_reg),
    .response_3       (response_3_reg)
    );

sd_cmd_serial_host cmd_serial_host0(
    .clock            (HCLK),
    .clock_posedge    (clock_posedge),
    .clock_data_in    (clock_data_in),
    .reset            (reset | ctrl_rst | go_idle),
    .setting          (cmd_setting),
    .cmd              (cmd),
    .start            (cmd_start_tx),
    .finish           (cmd_finish),
    .response         (cmd_response),
    .crc_ok           (cmd_crc_ok),
    .index_ok         (cmd_index_ok),
    .cmd_i            (sd_cmd_i),
    .cmd_o            (sd_cmd_o),
    .cmd_oe           (sd_cmd_oe)
    );

sd_data_master sd_data_master0(
    .clock            (HCLK),
    .clock_posedge    (clock_posedge),
    .reset            (reset | ctrl_rst),
    .start_tx         (data_start_tx),
    .start_rx         (data_start_rx),
    .timeout          (data_timeout_reg),
    .d_write          (d_write),
    .d_read           (d_read),
    .en_tx_fifo       (en_tx_fifo),
    .en_rx_fifo       (en_rx_fifo),
    .fifo_empty       (fifo_empty),
    .fifo_ready       (fifo_ready),
    .fifo_full        (fifo_full),
    .bus_cycle        (m_axi_cyc || m_axi_bresp_cnt != 0), // INAXI: Only direct mention of axi here
    .xfr_complete     (!data_busy),
    .crc_error        (!data_crc_ok),
    .bus_error        (m_bus_error),
    .int_status       (data_int_status),
    .int_status_rst   (data_int_rst)
    );

sd_data_serial_host sd_data_serial_host0(
    .clock            (HCLK),
    .clock_posedge    (clock_posedge),
    .clock_data_in    (clock_data_in),
    .reset            (reset | ctrl_rst),
    .data_in          (fifo_dout),
    .rd               (tx_fifo_re),
    .data_out         (data_in_rx_fifo),
    .we               (rx_fifo_we),
    .dat_oe           (sd_dat_oe),
    .dat_o            (sd_dat_o),
    .dat_i            (sd_dat_i),
    .blksize          (block_size_reg),
    .bus_4bit         (controller_setting_reg[0]),
    .blkcnt           (block_count_reg),
    .start            ({d_read, d_write}),
    .byte_alignment   (dma_addr_reg[1:0]),
    .sd_data_busy     (sd_data_busy),
    .busy             (data_busy),
    .crc_ok           (data_crc_ok)
    );

assign interrupt =
    |(cmd_int_status_reg & cmd_int_enable_reg) ||
    |(data_int_status_reg & data_int_enable_reg) ||
    (sd_insert_int & sd_insert_ie) ||
    (sd_remove_int & sd_remove_ie);

endmodule
