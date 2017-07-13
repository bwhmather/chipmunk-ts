import typescript from 'rollup-plugin-typescript2';
import uglify from 'rollup-plugin-uglify';


export default [
  {
    entry: 'src/index.ts',
    format: 'es',
    dest: 'dist/chipmunk.jsm',
    plugins: [
      typescript({
        abortOnError: false,
      }),
    ]
  },
  {
    entry: 'src/index.ts',
    format: 'iife',
    moduleName: 'cp',
    dest: 'dist/chipmunk.js',
    sourceMapFile: 'dist/chipmunk.js.map',
    sourceMap: true,
    plugins: [
      typescript({
        abortOnError: false,
      }),
    ]
  },
  {
    entry: 'src/index.ts',
    format: 'iife',
    moduleName: 'cp',
    dest: 'dist/chipmunk.min.js',
    sourceMapFile: 'dist/chipmunk.min.js.map',
    sourceMap: true,
    plugins: [
      typescript({
        abortOnError: false,
      }),
      uglify(),
    ]
  },
]


