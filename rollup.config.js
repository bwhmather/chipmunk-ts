import typescript from 'rollup-plugin-typescript2';
import uglify from 'rollup-plugin-uglify';


export default [
  {
    input: 'src/index.ts',
    plugins: [
      typescript({
        abortOnError: false,
      }),
    ],
    output: {
      format: 'es',
      file: 'dist/chipmunk.jsm',
    }
  },
  {
    input: 'src/index.ts',
    name: 'cp',
    sourcemap: true,
    plugins: [
      typescript({
        abortOnError: false,
      }),
    ],
    output: {
      format: 'iife',
      file: 'dist/chipmunk.js',
      sourcemapFile: 'dist/chipmunk.js.map',
    }
  },
  {
    input: 'src/index.ts',
    name: 'cp',
    sourcemap: true,
    plugins: [
      typescript({
        abortOnError: false,
      }),
      uglify(),
    ],
    output: {
      format: 'iife',
      file: 'dist/chipmunk.min.js',
      sourcemapFile: 'dist/chipmunk.min.js.map'
    }
  },
]


