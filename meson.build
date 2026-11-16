project(
    'power-capping',
    'cpp',
    default_options: ['warning_level=3', 'werror=true', 'cpp_std=c++23'],
    license: 'Apache-2.0',
    version: '1.0',
)

# Include directories
inc_dirs = include_directories('inc', meson.current_build_dir())

# Source files
src_files = [
  'src/power_cap.cpp',
  'src/main.cpp'
]

deps = [
    dependency('libgpiodcxx', default_options: ['bindings=cxx']),
    dependency('boost'),
    dependency('phosphor-dbus-interfaces'),
    dependency('phosphor-logging'),
    dependency('sdbusplus'),
    dependency('libsystemd'),
    dependency('nlohmann_json', include_type: 'system'),
]

# Extra libraries
extra_libs = [
  '-lstdc++fs',
  '-lphosphor_dbus',
  '-lapml64',
  '-lpthread',
  '-lm'
]

# Executable
executable(
  'power-capping',
  src_files,
  include_directories : inc_dirs,
  dependencies : deps,
  link_args : extra_libs,
  install : true
)

systemd = dependency('systemd')

# Install templated service file (one instance per host: @0, @1, @2)
install_data('service_files/xyz.openbmc_project.Control.Host.Power_cap@.service',
  install_dir : systemd.get_pkgconfig_variable('systemdsystemunitdir'),
)
