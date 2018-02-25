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
      sourcemap: true,
      sourcemapFile: 'dist/chipmunk.mjs.map'
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
      file: 'dist/chipmunk.umd.js',
      sourcemap: true,
      sourcemapFile: 'dist/chipmunk.umd.js.map'
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
      format: 'iife',
      name: 'cp',
      file: 'dist/chipmunk.js',
      sourcemap: true,
      sourcemapFile: 'dist/chipmunk.js.map',
    }
  },
  {
    input: 'src/index.ts',
    plugins: [
      typescript({
        abortOnError: false,
        useTsconfigDeclarationDir: true,
      }),
      uglify(),
    ],
    output: {
      format: 'iife',
      name: 'cp',
      file: 'dist/chipmunk.min.js',
      sourcemap: true,
      sourcemapFile: 'dist/chipmunk.min.js.map'
    }
  },
]


