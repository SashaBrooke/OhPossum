project('my_app', 'cpp',
    version: '1.0.0',
    default_options: [
        'cpp_std=c++17',
        'buildtype=release',
        'werror=true',
        'warning_level=3'
    ]
)

# Add the rpicam-apps subproject (with opencv and hailo enabled)
rpicam = subproject('rpicam-apps',
    default_options: [
        'enable_libav=enabled',
        'enable_drm=enabled',
        'enable_egl=enabled',
        'enable_qt=enabled',
        'enable_opencv=enabled',
        'enable_hailo=enabled',
    ]
)

# Get the rpicam library
rpicam_lib = rpicam.get_variable('rpicam_app')

# Get rpicam-apps includes
rpicam_includes = include_directories('subprojects/rpicam-apps')

# App source files
src_files = [
    'src/main.cpp',
]

# Get dependencies
libcamera_dep = dependency('libcamera', required: true)
dl_dep = dependency('dl', required: true)
boost_dep = dependency('boost', modules : ['program_options'])

# Create app executable
exe = executable('my_app',
    src_files,
    include_directories: [
        include_directories('include'),
        rpicam_includes
    ],
    dependencies: [
        libcamera_dep,
        dl_dep,
        boost_dep,
    ],
    link_with: rpicam_lib,
    install: true
)

# Print a summary
summary({
    'libcamera': libcamera_dep.version(),
    'dl': dl_dep.version(),
    'boost': boost_dep.version(),
    'rpicam-apps': 'subproject used',
}, bool_yn: true)
