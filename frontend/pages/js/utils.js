'use strict';

goog.provide('Blockly.utils');

/**
 * Get file content from the server.
 * @param {string} file relative path to the file on the web server.
 * @return {string} file content.
 */
Blockly.readFile = function(file) {
    var rawFile = new XMLHttpRequest();
    var code = "";
    rawFile.open("GET", file, false);
    rawFile.onreadystatechange = function ()
    {
        if(rawFile.readyState === 4)
        {
            if(rawFile.status === 200 || rawFile.status == 0)
            {
                code = rawFile.responseText;
            }
        }
    }
    rawFile.send(null);
    return code;
};