# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
# Do not print "Entering directory ...".
MAKEFLAGS += --no-print-directory
endif

OPENCM3_DIR ?= $(realpath libopencm3)

all: lib src

lib:
	$(Q)if [ ! "`ls -A $(OPENCM3_DIR)`" ] ; then \
		printf "######## ERROR ########\n"; \
		printf "\tlibopencm3 is not initialized.\n"; \
		printf "\tPlease run:\n"; \
		printf "\t$$ git submodule init\n"; \
		printf "\t$$ git submodule update\n"; \
		printf "\tbefore running make.\n"; \
		printf "######## ERROR ########\n"; \
		exit 1; \
		fi
	$(Q)$(MAKE) -C $(OPENCM3_DIR)

src: lib
	@printf "  BUILD   $@\n";
	$(Q)$(MAKE) --directory=$@ OPENCM3_DIR=$(OPENCM3_DIR)


clean:
	$(Q)$(MAKE) -C libopencm3 clean
	$(Q)$(MAKE) -C src clean

.PHONY: clean lib src
