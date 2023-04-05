const path = require("path");

module.exports = {
    entry: {
        app: './src/ros.ts'
    },
    output: {
        path: path.resolve(__dirname, 'dist'),
        filename: 'ros.js'
    },
    resolve: {
        extensions: ['.ts', '.tsx', '.js']
    },
    devtool: 'source-map',
    plugins: [

    ],
    module: {
        rules: [{
            test: /\.tsx?$/,
            loader: 'ts-loader',
            exclude: /node_modules/
        }]
    }
}