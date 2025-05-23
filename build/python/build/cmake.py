import os
import re
import subprocess
from typing import cast, Optional, Sequence, TextIO, Union
from collections.abc import Mapping

from build.project import Project
from .toolchain import AnyToolchain, Toolchain

def __write_cmake_compiler(f: TextIO, language: str, compiler: str) -> None:
    s = compiler.split(' ', 1)
    if len(s) == 2:
        print(f'set(CMAKE_{language}_COMPILER_LAUNCHER {s[0]})', file=f)
        compiler = s[1]
    print(f'set(CMAKE_{language}_COMPILER {compiler})', file=f)

def __write_cmake_toolchain_file(f: TextIO, toolchain: Toolchain, no_isystem: bool, cmake_system_name: str) -> None:
    cppflags = toolchain.cppflags
    if no_isystem:
        cppflags = re.sub(r'\s*-isystem\s+\S+\s*', ' ', cppflags)

    # Objective-C support here is a kludge; we use the C flags and the
    # C compiler, but that appears to be good enough for macOS / iOS.

    f.write(f"""
set(CMAKE_SYSTEM_NAME {cmake_system_name})
set(CMAKE_SYSTEM_PROCESSOR {toolchain.host_triplet.split('-', 1)[0]})

set(CMAKE_C_COMPILER_TARGET {toolchain.host_triplet})
set(CMAKE_CXX_COMPILER_TARGET {toolchain.host_triplet})
set(CMAKE_OBJC_COMPILER_TARGET {toolchain.host_triplet})

set(CMAKE_C_FLAGS_INIT "{toolchain.cflags} {cppflags}")
set(CMAKE_CXX_FLAGS_INIT "{toolchain.cxxflags} {cppflags}")
set(CMAKE_OBJC_FLAGS_INIT "{toolchain.cflags} {cppflags}")
""")
    __write_cmake_compiler(f, 'C', toolchain.cc)
    __write_cmake_compiler(f, 'CXX', toolchain.cxx)
    __write_cmake_compiler(f, 'OBJC', toolchain.cc)

    f.write(f"""
set(CMAKE_AR {toolchain.ar})
set(CMAKE_RANLIB {toolchain.ranlib})
""")

    if cmake_system_name == 'Darwin':
        # On macOS, cmake forcibly adds an "-isysroot" flag even if
        # one is already present in the flags variable; this breaks
        # cross-compiling for iOS, and can be worked around by setting
        # the CMAKE_OSX_SYSROOT variable
        # (https://cmake.org/cmake/help/latest/variable/CMAKE_OSX_SYSROOT.html).
        m = re.search(r'-isysroot +(\S+)', toolchain.cflags)
        if m:
            sysroot = m.group(1)

            print(f'set(CMAKE_OSX_SYSROOT {sysroot})', file=f)

            # search libraries and headers only in the sysroot, not on
            # the build host
            f.write(f"""
set(CMAKE_FIND_ROOT_PATH "{toolchain.install_prefix};{sysroot}")
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
""")

def configure(toolchain: AnyToolchain, src: str, build: str, args: list[str]=[], env: Optional[Mapping[str, str]]=None) -> None:
    cross_args: list[str] = []

    if toolchain.is_windows:
        cross_args.append('-DCMAKE_RC_COMPILER=' + cast(str, toolchain.windres))
    args = [arg.replace('${CMAKE_INSTALL_PREFIX}', toolchain.install_prefix) for arg in args]

    configure = [
        'cmake',
        src,

        '-DCMAKE_INSTALL_PREFIX=' + toolchain.install_prefix,
        '-DCMAKE_BUILD_TYPE=release',

        '-GNinja',
    ] + cross_args + args

    if toolchain.host_triplet is not None:
        # cross-compiling: write a toolchain file
        os.makedirs(build, exist_ok=True)

        # Several targets need a sysroot to prevent pkg-config from
        # looking for libraries on the build host (TODO: fix this
        # properly); but we must not do that on Android because the NDK
        # has a sysroot already
        no_isystem = False
        if not toolchain.is_android and not toolchain.is_darwin:
            configure.append('-DCMAKE_SYSROOT=' + toolchain.install_prefix)

            # strip "-isystem" to avoid build failures with C++ headers
            # because "#include_next" ceases to work
            no_isystem = True

        cmake_system_name = 'Linux'
        if toolchain.is_darwin:
            cmake_system_name = 'Darwin'
            if toolchain.is_target_ios and 'SDL2' in src:
                # SDL2 needs CMAKE_SYSTEM_NAME set to iOS, otherwise it will build for macOS
                # but OpenSSL needs CMAKE_SYSTEM_NAME set to Darwin, otherwise it will fail to build
                cmake_system_name = 'iOS'
        elif toolchain.is_windows:
            cmake_system_name = 'Windows'

        cmake_toolchain_file = os.path.join(build, 'cmake_toolchain_file')
        with open(cmake_toolchain_file, 'w') as f:
            __write_cmake_toolchain_file(f, toolchain, no_isystem, cmake_system_name)

        configure.append('-DCMAKE_TOOLCHAIN_FILE=' + cmake_toolchain_file)

    if env is None:
        env = toolchain.env
    else:
        env = {**toolchain.env, **env}

    print(configure)
    subprocess.check_call(configure, env=env, cwd=build)

class CmakeProject(Project):
    def __init__(self, url: Union[str, Sequence[str]], md5: str, installed: str,
                 configure_args: list[str]=[],
                 windows_configure_args: list[str]=[],
                 env: Optional[Mapping[str, str]]=None,
                 **kwargs):
        Project.__init__(self, url, md5, installed, **kwargs)
        self.configure_args = configure_args
        self.windows_configure_args = windows_configure_args
        self.env = env

    def configure(self, toolchain: AnyToolchain) -> str:
        src = self.unpack(toolchain)
        build = self.make_build_path(toolchain)
        configure_args = self.configure_args
        if toolchain.is_windows:
            configure_args = configure_args + self.windows_configure_args
        configure(toolchain, src, build, configure_args, self.env)
        return build

    def _build(self, toolchain: AnyToolchain, target_toolchain: Optional[AnyToolchain]=None) -> None:
        build = self.configure(toolchain)
        subprocess.check_call(['ninja', '-v', 'install'],
                              cwd=build, env=toolchain.env)
