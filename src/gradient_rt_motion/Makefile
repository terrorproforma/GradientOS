CXX ?= g++

TARGET := gradient-rt-motion
SRCS := main.cpp

# Keep this self-contained for RevPi bring-up (no external deps yet).
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic -D_GNU_SOURCE
LDFLAGS ?= -pthread

# Optional: enable IgH libecrt (libethercat) if installed.
# This controls both:
# - compile-time enable (GRADIENT_ECRT_ENABLED)
# - link flags (-lethercat, include/lib paths, etc.)
PKG_CONFIG ?= pkg-config
ECRT_CFLAGS := $(shell $(PKG_CONFIG) --cflags libethercat 2>/dev/null)
ECRT_LIBS := $(shell $(PKG_CONFIG) --libs libethercat 2>/dev/null)
ECRT_LIBDIR := $(shell $(PKG_CONFIG) --variable=libdir libethercat 2>/dev/null)

ifneq ($(strip $(ECRT_CFLAGS)$(ECRT_LIBS)),)
  CXXFLAGS += -DGRADIENT_ECRT_ENABLED=1 $(ECRT_CFLAGS)
  # Make the binary runnable even if libethercat lives outside the system linker paths
  # (common when IgH is installed to /usr/local).
  ifneq ($(strip $(ECRT_LIBDIR)),)
    LDFLAGS += -Wl,-rpath,$(ECRT_LIBDIR)
  endif
  LDFLAGS += $(ECRT_LIBS)
else
  CXXFLAGS += -DGRADIENT_ECRT_ENABLED=0
endif

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRCS) ipc_v1.hpp ecrt_detect.hpp ds402.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $@ $(LDFLAGS)

clean:
	rm -f $(TARGET)

