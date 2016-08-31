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


/**
 * Call web service.
 * @param {string} file relative path to the file on the web server.
 */
Blockly.callWebService = function(module, method, parameters, asynchronous, success_callback, error_callback) {
    result = $.ajax({
        url: 'web_service_module',
        type: 'post',
        dataType: 'json',
        async: asynchronous,
        success: function(data, textStatus, jqXHR) {
            if ('error' in data) {
                if (error_callback) {
                    error_callback(jqXHR, textStatus, data['error']);
                }
            }
            else {
                if (success_callback) {
                    success_callback(data);
                }
            }
        },
        error: error_callback,
        data: JSON.stringify({
            module: module,
            method: method,
            parameters: parameters
        })
    });
    return result;
}
