# Test compilation makefile
#
# Copyright (c) 2016 Ryan Kurte
# This file is covered under the MIT license available at: https://opensource.org/licenses/MIT
# Modified by balena

.PHONY: setup bgm13 bgm1b

setup:
	git submodule update --init; mkdir -p builds;

# BGM13
bgm13:
	cd builds; rm -rf $@; mkdir $@; cd $@; cmake -DDEVICE=BGM13P22F512GA ../..; make;

# BGM1B (BalenaFin v1.0 & v1.1)
bgm1b:
	cd builds; rm -rf $@; mkdir $@; cd $@; cmake -DDEVICE=EFR32BG1B232F256GM48 ../..; make;

clean:
	rm -rf builds
