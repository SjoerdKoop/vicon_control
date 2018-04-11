# Binary output path
BIN_PATH = ./bin

# Project paths
PROJECTS = $(wildcard src/*)

default: $(PROJECTS)

$(PROJECTS): .FORCE
	@ # Makes all projects in src/
	@ echo "Making $(shell basename $@)"
	@ $(MAKE) --no-print-directory -C $@

.FORCE: