project('otarover', ['c'],
  version: '0.1',
  license: [
        'GPL-3.0',
  ],
  default_options : [ 'warning_level=1',
                      'buildtype=debugoptimized' ])

cc = meson.get_compiler('c')

cdata = configuration_data()

check_headers = [
  ['HAVE_INTTYPES_H', 'inttypes.h'],
]

foreach h : check_headers
  if cc.has_header(h.get(1))
    cdata.set(h.get(0), 1)
  endif
endforeach

check_functions = [
  # check token HAVE_GETTEXT
  ['HAVE_GETTEXT', 'gettext', true],
]

foreach f : check_functions
  if cc.has_function(f.get(1))
    cdata.set(f.get(0), 1)
  else
    if f.get(2) == true
      error('@0@: not found'.format(f.get(1)))
    endif
  endif
endforeach


platform = get_option('platform')
system = get_option('system')

cdata.set('PLATFORM', '"@0@"'.format(platform))
cdata.set('SYSTEM', '"@0@"'.format(system))
cdata.set('PREFIX', '"@0@"'.format(get_option('prefix')))
cdata.set('LIBDIR', '"@0@"'.format(get_option('libdir')))
cdata.set('DATADIR', '"@0@"'.format(get_option('datadir')))

configure_file(output : 'config.h', configuration : cdata)

compiler = meson.get_compiler('c')

otarover_platform_src = [
]

otarover_lib_platform_src = [
]

if platform == 'blue' and system == 'linux'
  message('Platform ' + platform + ' selected')
  otarover_platform_src = [
    #'src/platform/linux/blue/platform_linux_blue.c'
  ]

  otarover_lib_platform_src = [
  ]

else
  error('No platform/system selected. Use -Dplatform=<platform> -Dsystem=<system> to set one')
endif

otarover_deps = [
]

otarover_lib_sources = [
  'lib/otaroverlib.c'
]

otarover_sources = [
  'src/main.c',
  'src/net/otarover_protocol.c',
  otarover_platform_src
]

incdir = include_directories('inc')

otarover_cli_sources = [
  'src/tools/cli/main.c'
]

otarover_cli_deps = [
]

static_lib = static_library('otarover', otarover_lib_sources, include_directories: incdir)
shared_lib = shared_library('otarover', otarover_lib_sources, include_directories: incdir)

exe = executable('otaroverctl', otarover_cli_sources, dependencies : otarover_cli_deps, include_directories: incdir, install : true, link_with: static_lib)
test('Check Generated Application', exe)
exe = executable('otarover', otarover_sources, dependencies : otarover_deps, include_directories: incdir, install : true, link_with: static_lib)
test('Check Generated Application', exe)
