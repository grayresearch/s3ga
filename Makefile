# S3GA: simple scalable serial FPGA
# By Jan Gray. Copyright (C) 2021-2022 Gray Research LLC. All rights reserved.
#
# Adopted from the cocotb tutorial from Matt Venn's Zero-to-ASIC class.

export PYTHONPATH := test:$(PYTHONPATH)
export LIBPYTHON_LOC=$(shell cocotb-config --libpython)

all: test_pipe test_cfg_ram

test_pipe:
	rm -rf sim; mkdir sim
	iverilog -o sim/sim.vvp -Isrc -s pipe -s dump -g2012 src/s3ga.v test/dump_pipe.v
	MODULE=test.test_pipe vvp -M $$(cocotb-config --prefix)/cocotb/libs -m libcocotbvpi_icarus sim/sim.vvp
	! grep failure results.xml

test_cfg_ram:
	rm -rf sim; mkdir sim
	iverilog -o sim/sim.vvp -Isrc -s cfg_ram -s dump -g2012 src/s3ga.v test/dump_cfg_ram.v
	MODULE=test.test_cfg_ram vvp -M $$(cocotb-config --prefix)/cocotb/libs -m libcocotbvpi_icarus sim/sim.vvp
	! grep failure results.xml

clean:
	rm -rf *.vcd sim test/__pycache__ results.xml

