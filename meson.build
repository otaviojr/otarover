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

cdata.set('PLATFORM', '"@0@"'.format(platform))
cdata.set('PREFIX', '"@0@"'.format(get_option('prefix')))
cdata.set('LIBDIR', '"@0@"'.format(get_option('libdir')))
cdata.set('DATADIR', '"@0@"'.format(get_option('datadir')))

configure_file(output : 'config.h', configuration : cdata)

compiler = meson.get_compiler('c')

platform_src = [
                ]

if platform == 'blue'
  message('Platform ' + platform + ' selected')
  platform_src = [
		     'src/platform/blue/platform_blue.c'
                   ]
else
  error('No platform selected. Use -Dplatform=<platform> to set one')
endif

deps = [
       ]

sources = [
		'src/main.c',
                platform_src
          ]

incdir = include_directories('inc')

exe = executable('otarover', sources, dependencies : deps, include_directories: incdir, install : true)

test('Check Generated Application', exe)
