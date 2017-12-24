project('otarover', ['c'],
  version: '0.1',
  license: [
        'GPL-2.0',
  ],
  default_options : [ 'warning_level=1',
                      'buildtype=debugoptimized' ])

cc = meson.get_compiler('c')

sources = [
		'src/main.c'
          ]

deps = 	[
	]

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

cdata.set('PREFIX', '"@0@"'.format(get_option('prefix')))
cdata.set('LIBDIR', '"@0@"'.format(get_option('libdir')))
cdata.set('DATADIR', '"@0@"'.format(get_option('datadir')))

configure_file(output : 'config.h', configuration : cdata)

compiler = meson.get_compiler('c')

exe = executable('otarover', sources, dependencies : deps, install : true)

test('Check Generated Application', exe)
