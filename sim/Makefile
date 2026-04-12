RTL_DIR  := ../rtl/dendrite
TB_DIR   := ../tb
BUILD    := build

VERILATOR := verilator
VLFLAGS   := --binary --trace-fst -Wno-fatal --timing --language 1800-2017

RTL_SRC := $(RTL_DIR)/pe.sv            \
           $(RTL_DIR)/systolic_array.sv \
           $(RTL_DIR)/vector_regfile.sv \
           $(RTL_DIR)/warp_scheduler.sv \
           $(RTL_DIR)/divergence_predictor.sv \
           $(RTL_DIR)/coprocessor_if.sv \
           $(RTL_DIR)/dendrite_top.sv

# ─── Targets ─────────────────────────────────────────────────

.PHONY: all systolic warp top lint clean

all: systolic warp top

# Test systolic array in isolation
systolic: $(TB_DIR)/tb_systolic_array.sv $(RTL_SRC) | $(BUILD)
	$(VERILATOR) $(VLFLAGS) --top-module tb_systolic_array \
		--Mdir $(BUILD)/$@ -I$(RTL_DIR) \
		$(RTL_DIR)/pe.sv $(RTL_DIR)/systolic_array.sv \
		$(TB_DIR)/tb_systolic_array.sv
	$(BUILD)/$@/Vtb_systolic_array
	@echo "=== systolic array test done ==="

# Test warp scheduler + divergence predictor
warp: $(TB_DIR)/tb_warp_scheduler.sv $(RTL_SRC) | $(BUILD)
	$(VERILATOR) $(VLFLAGS) --top-module tb_warp_scheduler \
		--Mdir $(BUILD)/$@ -I$(RTL_DIR) \
		$(RTL_DIR)/pe.sv $(RTL_DIR)/systolic_array.sv \
		$(RTL_DIR)/warp_scheduler.sv $(RTL_DIR)/divergence_predictor.sv \
		$(TB_DIR)/tb_warp_scheduler.sv
	$(BUILD)/$@/Vtb_warp_scheduler
	@echo "=== warp scheduler test done ==="

# Full integration test
top: $(TB_DIR)/tb_dendrite_top.sv $(RTL_SRC) | $(BUILD)
	$(VERILATOR) $(VLFLAGS) --top-module tb_dendrite_top \
		--Mdir $(BUILD)/$@ -I$(RTL_DIR) \
		$(RTL_SRC) \
		$(TB_DIR)/tb_dendrite_top.sv
	$(BUILD)/$@/Vtb_dendrite_top
	@echo "=== dendrite top test done ==="

# Lint only (no sim)
lint:
	$(VERILATOR) --lint-only -Wall --timing -I$(RTL_DIR) $(RTL_SRC)

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD) *.fst
