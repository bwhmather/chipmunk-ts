import typescript from 'rollup-plugin-typescript';

export default [
  {
    entry: 'lib/index.ts',
    format: 'es',
    dest: 'dist/chipmunk.jsm',
    plugins: [
      typescript(),
    ]
  },
]


