# Test compilation makefile
#
# Copyright (c) 2016 Ryan Kurte
# This file is covered under the MIT license available at: https://opensource.org/licenses/MIT

.PHONY: setup balenafin devkit efm32g efm32bg efm32gg efm32zg efm32wg efm32lg efm32tg efm32hg ezr32lg ezr32wg efm32pg efr32fg

test: setup balenafin devkit efm32g efm32bg efm32gg efm32zg efm32wg efm32lg efm32tg efm32hg ezr32lg ezr32wg efm32pg efr32fg 

all: setup balenafin devkit

setup:
	mkdir -p $(OUTDIR)

#BalenaFin
balenafin:
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EFR32BG1B232F256GM48 -DSERIAL=FIN -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make
	
#Silabs-BRD4300A
devkit:
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EFR32BG1B232F256GM48 -DSERIAL=DEV -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make

#EFM32G Family
efm32g:
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EFM32G210F128 -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make

#EFM32GG Family
efm32bg:
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EFR32BG1B232F256GM48 -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make

#EFM32GG Family
efm32gg:
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EFM32GG990F1024 -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make

#EFM32ZG Family
efm32zg:
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EFM32ZG222F32 -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make

#EFM32WG Family
efm32wg:
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EFM32WG990F256 -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make

#EFM32LG Family
efm32lg:
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EFM32LG990F256 -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make

#EFM32TG Family
efm32tg:
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EFM32TG840F32 -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make

#EFM32WG Family
efm32hg:
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EFM32HG322F64 -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make

#EFM32LG Family
ezr32lg:
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EZR32LG230F256R67 -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make

#EFM32TG Family
ezr32wg:
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EZR32WG230F256R67 -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make

#efm32pg Family
efm32pg:
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EFM32PG1B200F256GM48 -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make

#EFR32FG Family
efr32fg:
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EFR32FG13P231F512GM48 -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make

#EFR32MG Family
efr32mg:
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EFR32MG12P332F1024GL125 -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make

clean:
	rm -rf $(OUTDIR)