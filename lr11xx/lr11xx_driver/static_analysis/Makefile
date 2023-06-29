SOURCE_FOLDER ?= ../src
BUILD_FOLDER ?= ./obj

SRC_FILES = $(wildcard $(SOURCE_FOLDER)/*.c)
OBJ_FILES = $(addprefix $(BUILD_FOLDER)/,$(notdir $(SRC_FILES:.c=.o)))
INCLUDES = -I../src

CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -fdata-sections -ffunction-sections
CFLAGS += -fstack-usage
CFLAGS += -std=c99 -Og -g
CWARNING += \
-Wall \
-Wextra \
-Wpedantic \
-Wdouble-promotion \
-Wignored-qualifiers \
-Wswitch-default \
-Wswitch-enum \
-Wunused-parameter \
-Wfloat-equal \
-Wconversion \
-Wsign-conversion \
-Wmissing-prototypes \
-Wmissing-declarations \
-Wredundant-decls \
-Wunused-macros

LDFLAGS +=

LIB_NAME = liblr11xxdriver.so

DIRECTORY_BUILDER = @mkdir -p $(BUILD_FOLDER)

.PHONY: clean lib distclean

lib: $(LIB_NAME)


$(LIB_NAME): $(OBJ_FILES)
	$(CC) -o $@ $^ -shared $(LDFLAGS)

$(BUILD_FOLDER)/%.o: $(SOURCE_FOLDER)/%.c
	$(DIRECTORY_BUILDER)
	$(CC) -o $@ $< -c -fPIC $(CWARNING) $(CFLAGS) $(INCLUDES)

clean:
	rm -rf $(BUILD_FOLDER)

distclean: clean
	rm -f $(LIB_NAME)
