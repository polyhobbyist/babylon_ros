const path = require("path");

module.exports = {
    entry: './src/ros.ts',
    output: {
        path: path.resolve(__dirname, 'dist'),
        filename: 'babylon_ros.js',
        library: 'babylon_ros'
    },
    resolve: {
        extensions: ['.ts', '.tsx', '.js'],
        fallback: {
            // Webpack 5 no longer polyfills Node.js core modules automatically.
            // see https://webpack.js.org/configuration/resolve/#resolvefallback
            // for the list of Node.js core module polyfills.
            "stream": require.resolve("stream-browserify"),
            "timers": require.resolve("timers-browserify")
        }
    },
    devtool: 'source-map',
    plugins: [

    ],
    module: {
        rules: [{
            test: /\.tsx?$/,
            exclude: /node_modules/,
            use: [{
                loader: 'ts-loader',
            }]
    
        }]
    }
}