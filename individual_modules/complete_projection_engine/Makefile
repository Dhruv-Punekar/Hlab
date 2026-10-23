# Simulation with Icarus Verilog (run from this folder)
#   make all        run every testbench
#   make engine     run one testbench (addr | engine | uart | top)
#   make wave T=top open a waveform in GTKWave
#   make mem IMG=photo.png   regenerate mem/ files from an image

IV   = iverilog -g2005 -Wall
RTL  = rtl/pixel_addr_gen.v rtl/row_projection.v rtl/col_projection.v \
       rtl/projection_engine.v rtl/image_rom.v rtl/uart_tx.v \
       rtl/result_sender.v rtl/debounce.v rtl/top.v
OUT  = sim_out

.PHONY: all addr engine uart top wave mem clean

all: addr engine uart top

$(OUT):
	mkdir -p $(OUT)

addr: | $(OUT)
	$(IV) -s tb_pixel_addr_gen -o $(OUT)/tb_pixel_addr_gen rtl/pixel_addr_gen.v tb/tb_pixel_addr_gen.v
	vvp $(OUT)/tb_pixel_addr_gen

engine: | $(OUT)
	$(IV) -s tb_projection_engine -o $(OUT)/tb_projection_engine $(RTL) tb/tb_projection_engine.v
	vvp $(OUT)/tb_projection_engine

uart: | $(OUT)
	$(IV) -s tb_uart_tx -o $(OUT)/tb_uart_tx rtl/uart_tx.v tb/tb_uart_tx.v
	vvp $(OUT)/tb_uart_tx

top: | $(OUT)
	$(IV) -s tb_top -o $(OUT)/tb_top $(RTL) tb/tb_top.v
	vvp $(OUT)/tb_top

wave:
	gtkwave tb_$(T).vcd

mem:
	python3 scripts/img2mem.py $(IMG) --out mem

clean:
	rm -rf $(OUT) *.vcd
