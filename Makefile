# S3GA: simple scalable serial FPGA
# By Jan Gray. Copyright (C) 2021-2022 Gray Research LLC.

# SPDX-FileCopyrightText: 2022 Gray Research LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# SPDX-License-Identifier: Apache-2.0

# Adopted from the cocotb tutorial from Matt Venn's Zero-to-ASIC class.

export PYTHONPATH := test:$(PYTHONPATH)
export LIBPYTHON_LOC=$(shell cocotb-config --libpython)

all: test_pipe test_cfg_ram test_xbar test_switch test_lb test_iob

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

test_xbar:
	rm -rf sim; mkdir sim
	iverilog -o sim/sim.vvp -Isrc -s xbar -s dump -g2012 src/s3ga.v test/dump_xbar.v
	MODULE=test.test_xbar vvp -M $$(cocotb-config --prefix)/cocotb/libs -m libcocotbvpi_icarus sim/sim.vvp
	! grep failure results.xml

test_switch:
	rm -rf sim; mkdir sim
	iverilog -o sim/sim.vvp -Isrc -s switch -s dump -g2012 src/s3ga.v test/dump_switch.v
	MODULE=test.test_switch vvp -M $$(cocotb-config --prefix)/cocotb/libs -m libcocotbvpi_icarus sim/sim.vvp
	! grep failure results.xml

test_lb:
	rm -rf sim; mkdir sim
	iverilog -o sim/sim.vvp -Isrc -s lb -s dump -g2012 src/s3ga.v test/dump_lb.v
	MODULE=test.test_lb vvp -M $$(cocotb-config --prefix)/cocotb/libs -m libcocotbvpi_icarus sim/sim.vvp
	! grep failure results.xml

test_iob:
	rm -rf sim; mkdir sim
	iverilog -o sim/sim.vvp -Isrc -s iob -s dump -g2012 src/s3ga.v test/dump_iob.v
	MODULE=test.test_iob vvp -M $$(cocotb-config --prefix)/cocotb/libs -m libcocotbvpi_icarus sim/sim.vvp
	! grep failure results.xml

test_s3ga:
	rm -rf sim; mkdir sim
	iverilog -o sim/sim.vvp -Isrc -s s3ga -s dump -g2012 src/s3ga.v test/dump_s3ga.v
	MODULE=test.test_s3ga vvp -M $$(cocotb-config --prefix)/cocotb/libs -m libcocotbvpi_icarus sim/sim.vvp
	! grep failure results.xml

clean:
	rm -rf *.vcd sim test/__pycache__ results.xml

