# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(PROJECT_PATH)/components/airkiss/include $(PROJECT_PATH)/components/airkiss/airkiss/include
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/airkiss $(PROJECT_PATH)/components/airkiss/lib/libairkiss.a  
COMPONENT_LINKER_DEPS += $(PROJECT_PATH)/components/airkiss/lib/libairkiss.a
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += airkiss
COMPONENT_LDFRAGMENTS += 
component-airkiss-build: 
