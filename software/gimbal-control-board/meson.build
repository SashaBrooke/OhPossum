project('my_app', 'cpp',
  version: '1.0.0',
  default_options: ['cpp_std=c++17', 'buildtype=release', 'werror=true', 'warning_level=3']
)

src_files = ['src/main.cpp']

libcamera_dep = dependency('libcamera', required: true)
dl_dep = dependency('dl', required: true)
boost_dep = dependency('boost', required: true)
rpicam_dep = dependency('rpicam_app', required: true)

boost_program_options_dep = declare_dependency(
  link_args: ['-lboost_program_options']
)

rpicam_include_dirs = include_directories(
    '/usr/local/include/rpicam-apps'
)

exe = executable('my_app',
  src_files,
  include_directories: ['include'] + rpicam_include_dirs,
  dependencies: [
    libcamera_dep,
    dl_dep,
    boost_dep,
    boost_program_options_dep,
    rpicam_dep,
  ],
  install: true
)
