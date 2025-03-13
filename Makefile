SHELL := /bin/sh

# Communication-related
COMMUNICATION_FOLDER_PATH := ./communication
HOSTS_JSON := $(COMMUNICATION_FOLDER_PATH)/hosts.json
COMMUNICATOR_PY := $(COMMUNICATION_FOLDER_PATH)/socket/communicator.py
TEMP_PY_FILES_DIR := ./temp_py_files

.PHONY: all

all: run update communication

run:
	echo "Running"

update:
	venv/bin/pip freeze > requirements.txt

communication: generate_temp_py_files
	@echo "Running generated Python files..."
	@for file in $(TEMP_PY_FILES_DIR)/*.py; do \
		python $$file; \
	done

generate_temp_py_files:
	@echo "Generating temporary Python files..."
	@mkdir -p $(TEMP_PY_FILES_DIR)
	@for identifier in $(shell jq -r 'keys[]' $(HOSTS_JSON)); do \
		cp $(COMMUNICATOR_PY) $(TEMP_PY_FILES_DIR)/run_$$identifier.py; \
		sed -i '' 's/IDENTIFIER = \".*\"/IDENTIFIER = \"$$identifier\"/' $(TEMP_PY_FILES_DIR)/run_$$identifier.py; \
		echo "Generated $(TEMP_PY_FILES_DIR)/run_$$identifier.py"; \
	done

clean:
	rm -rf $(TEMP_PY_FILES_DIR)