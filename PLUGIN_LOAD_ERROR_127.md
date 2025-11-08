# Plugin Load Error 127 - Troubleshooting

## Error Message
```
[warn src/plugin.cpp:237 loadPlugin] Could not load plugin ...AmbushedCat:
Failed to load library ...AmbushedCat/plugin.dll: code 127
```

## What is Error Code 127?

On Windows, error code 127 (`ERROR_PROC_NOT_FOUND`) typically means:
1. **Missing dependency DLL** - A DLL that plugin.dll depends on cannot be found
2. **Missing function export** - A required function is not exported from a dependency
3. **Architecture mismatch** - 32-bit/64-bit mismatch (unlikely with Rack SDK)

## Common Causes for Link/RtMidi Plugin

### 1. Missing Windows System DLLs

The plugin requires these Windows system libraries:
- `ws2_32.dll` - Winsock 2 (Windows Sockets API)
- `iphlpapi.dll` - IP Helper API
- `winmm.dll` - Windows Multimedia API (for MIDI)

**These should be present on all Windows systems**, but verify:

```cmd
where ws2_32.dll
where iphlpapi.dll
where winmm.dll
```

### 2. Static vs Dynamic Linking

Check Makefile settings:

**Current settings (should be correct):**
```makefile
# Static libstdc++ to avoid ABI issues
LDFLAGS += -static-libstdc++

# Static pthread to avoid DLL dependency
LDFLAGS += -Wl,-Bstatic -lpthread -Wl,-Bdynamic

# System libraries (ws2_32, iphlpapi, winmm) - these are dynamic by default
LDFLAGS += -lwinmm -lws2_32 -liphlpapi
```

### 3. C++ Runtime

The `-static-libstdc++` flag should statically link the C++ standard library to avoid requiring `libstdc++-6.dll`.

Verify by checking if these are NOT in the plugin directory:
- ❌ `libstdc++-6.dll` (should NOT be needed - statically linked)
- ❌ `libgcc_s_seh-1.dll` (should NOT be needed - statically linked)
- ❌ `libwinpthread-1.dll` (should NOT be needed - statically linked)

## Solution Steps

### Step 1: Verify Plugin Size

The new plugin with Link/RtMidi should be ~45-50MB:

```bash
ls -lh plugin.dll
# Should show: ~47MB

ls -lh "C:/Users/Legit/AppData/Local/Rack2/plugins-win-x64/AmbushedCat/plugin.dll"
# Should ALSO show: ~47MB (after copying)
```

If the installed version is smaller (e.g., 2.8MB), it's the old version without Link:

```bash
# Copy the new build
cp plugin.dll "C:/Users/Legit/AppData/Local/Rack2/plugins-win-x64/AmbushedCat/plugin.dll"
```

### Step 2: Check Rack is Closed

VCV Rack locks the plugin DLL when running. **Close VCV Rack completely** before copying:

1. Close all Rack windows
2. Check Task Manager - ensure no `Rack.exe` process
3. Copy the new plugin.dll
4. Start Rack

### Step 3: Check for Export Conflicts

If using an older version of VCV Rack or the SDK, there might be symbol conflicts.

**Verify Rack version:**
- Rack 2.x (2.0.0+) required
- Check: Help → About in VCV Rack

### Step 4: Rebuild with Verbose Output

```bash
# Clean build with verbose output
mingw32-make clean
mingw32-make V=1 2>&1 | tee build.log

# Check for link errors
grep -i "error\|warning" build.log
```

Look for:
- ❌ "undefined reference to" - missing symbol
- ❌ "cannot find -l" - missing library
- ✅ "-lws2_32 -liphlpapi -lwinmm" present in link command

### Step 5: Dependency Walker (Advanced)

If you have Dependency Walker or similar tool:

```
depends.exe plugin.dll
```

This will show:
- All DLL dependencies
- Missing DLLs (red)
- Missing exports (red)

## Quick Fix: Try Linking System Libraries Statically

If the issue persists, try forcing static linking of Windows libraries:

**Edit Makefile, find the Windows section:**

```makefile
ifdef ARCH_WIN
    CXXFLAGS += -D__WINDOWS_MM__
    CXXFLAGS += -DLINK_PLATFORM_WINDOWS

    # Try static linking
    LDFLAGS += -Wl,-Bstatic -lwinmm -lws2_32 -liphlpapi -Wl,-Bdynamic
endif
```

Then rebuild:
```bash
mingw32-make clean
mingw32-make -j4
cp plugin.dll "C:/Users/Legit/AppData/Local/Rack2/plugins-win-x64/AmbushedCat/plugin.dll"
```

**Note:** Windows system DLLs (ws2_32, iphlpapi, winmm) are typically **not** statically linkable. They must be dynamically linked. The above is just for testing.

## Known Working Configuration

**Makefile (lines 47-54):**
```makefile
ifdef ARCH_WIN
    # Windows: Use WinMM MIDI API
    CXXFLAGS += -D__WINDOWS_MM__
    LDFLAGS += -lwinmm

    # Link platform detection
    CXXFLAGS += -DLINK_PLATFORM_WINDOWS
    LDFLAGS += -lws2_32 -liphlpapi
endif
```

**Later in Makefile (lines 89-95):**
```makefile
ifdef ARCH_WIN
    # Static libstdc++ to avoid DLL dependency
    LDFLAGS += -static-libstdc++

    # Static pthread to avoid DLL dependency
    LDFLAGS := $(filter-out -lpthread,$(LDFLAGS))
    LDFLAGS += -Wl,-Bstatic -lpthread -Wl,-Bdynamic
endif
```

## If All Else Fails

### Fallback: Remove Link Temporarily

To verify RtMidi alone works, temporarily disable Link:

1. Comment out Link code in `src/UsbSync.cpp`:
   ```cpp
   // #include <ableton/Link.hpp>
   // link = std::make_unique<ableton::Link>(120.0);
   ```

2. Rebuild and test:
   ```bash
   mingw32-make -j4
   cp plugin.dll "C:/Users/Legit/AppData/Local/Rack2/plugins-win-x64/AmbushedCat/plugin.dll"
   ```

If this works, the issue is specifically with Link. If it still fails, the issue is with RtMidi or the build configuration.

## Expected Behavior

When working correctly:
1. ✅ plugin.dll ~47MB
2. ✅ Rack loads plugin without errors
3. ✅ UsbSync module appears in module browser
4. ✅ Can add UsbSync to patch
5. ✅ MIDI port selector shows MIDI devices

## Additional Resources

- VCV Rack Plugin Development Tutorial: https://vcvrack.com/manual/PluginDevelopmentTutorial
- MinGW Static Linking: https://gcc.gnu.org/onlinedocs/gcc/Link-Options.html
- Windows DLL Search Order: https://docs.microsoft.com/en-us/windows/win32/dlls/dynamic-link-library-search-order

---

**Issue**: Error code 127 loading plugin.dll
**Likely Cause**: Missing dependency or old plugin.dll not replaced
**First Fix**: Copy new plugin.dll to Rack plugins directory
**Status**: Investigating

**Date**: November 8, 2024
