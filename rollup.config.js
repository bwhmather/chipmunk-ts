import typescript from 'rollup-plugin-typescript2';
import uglify from 'rollup-plugin-uglify';


export default [
  {
    input: 'src/index.ts',
    plugins: [
      typescript({
        abortOnError: false,
        useTsconfigDeclarationDir: true,
      }),
    ],
    output: {
      format: 'es',
      file: 'dist/chipmunk.mjs',
    }
  },
  {
    input: 'src/index.ts',
    plugins: [
      typescript({
        abortOnError: false,
        useTsconfigDeclarationDir: true,
      }),
    ],
    output: {
      format: 'cjs',
      file: 'dist/chipmunk.umd.js'
    }
  },
  {
    input: 'src/index.ts',
    name: 'cp',
    sourcemap: true,
    plugins: [
      typescript({
        abortOnError: false,
        useTsconfigDeclarationDir: true,
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
        useTsconfigDeclarationDir: true,
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


