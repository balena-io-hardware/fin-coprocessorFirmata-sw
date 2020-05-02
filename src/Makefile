.PHONY: all

CORES=1

all: balena devkit

config: set-cores
ifndef OUTDIR
export OUTDIR=builds
endif

set-cores:
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
	CORES=$(nproc)
endif
ifeq ($(UNAME_S),Darwin)
	CORES=$(sysctl -n hw.ncpu)
endif

#BalenaFin
balena: config
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EFR32BG1B232F256GM48 -DSERIAL=FIN -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make -j $(CORES)
	
#Silabs-BRD4300A
devkit: config
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EFR32BG1B232F256GM48 -DSERIAL=DEV -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make -j $(CORES)

clean:
	rm -rf $(OUTDIR)
