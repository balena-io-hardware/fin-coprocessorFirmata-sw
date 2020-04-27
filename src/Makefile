.PHONY: all

all: balena devkit

check-output:
ifndef OUTDIR
export OUTDIR=builds
endif

#BalenaFin
balena: check-output
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EFR32BG1B232F256GM48 -DSERIAL=FIN -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make
	
#Silabs-BRD4300A
devkit: check-output
	rm -rf $(OUTDIR)/$@; mkdir $(OUTDIR)/$@; cmake -DDEVICE=EFR32BG1B232F256GM48 -DSERIAL=DEV -S. -B$(OUTDIR)/$@; cd $(OUTDIR)/$@; make

clean:
	rm -rf $(OUTDIR)
