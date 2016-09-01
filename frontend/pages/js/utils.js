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
 * @param {string} module module name.
 * @param {string} method method name.
 * @param {object} parameters single object with all parameters either array or dictionary.
 * @param {boolean} asynchronous make call asynchronous.
 * @param {function} success_callback success callback.
 * @param {function} error_callback error callback.
 */
Blockly.callWebService = function(module, method, parameters, asynchronous, success_callback, error_callback) {
  var result = $.ajax({
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

/**
 * Asynchronously call web service.
 * @param {string} module module name.
 * @param {string} method method name.
 * @param {object} parameters single object with all parameters either array or dictionary.
 * @param {function} success_callback success callback.
 * @param {function} error_callback error callback.
 */

Blockly.asyncCallWebService = function(module, method, parameters, success_callback, error_callback) {
  return Blockly.callWebService(module, method, parameters, true, success_callback, error_callback)
}

/**
 * Global variable for toolbox XML.
 */

if (null == Blockly.toolboxXmlText) {
  Blockly.toolboxXmlText = "";
}

/**
 * Append toolbox xml to already existing during initialization of the workspace.
 * @param {string} xml code of the toolbox to append to already existing one.
 */
Blockly.appendToToolbox = function(xml) {
  Blockly.toolboxXmlText += xml;
}

/**
 * Return toolbox XML during initialization of the workspace.
 */
Blockly.getToolboxXmlText = function() {
  return '<xml xmlns="http://www.w3.org/1999/xhtml">' + Blockly.toolboxXmlText + '</xml>';
}

/**
 * Global variable with wait list of the calls
 */
if (null == Blockly.waitForComponentsList) {
  Blockly.waitForComponentsList = [];
}

/**
 * Asyncronously call web service to load component and wait for it before loading workspace.
 * @param {string} module module name.
 * @param {string} method method name.
 * @param {object} parameters single object with all parameters either array or dictionary.
 * @param {function} success_callback success callback.
 * @param {function} error_callback error callback.
 */
Blockly.waitForComponentToLoad = function(module, method, parameters, success_callback, error_callback) {
  Blockly.waitForComponentsList.push(
    Blockly.asyncCallWebService(module, method, parameters, success_callback, error_callback));
}

/**
 * Calls workspace_loading_function after all initialization is done.
 * @param {function} workspace_loading_function callback which actually loads workspace
 */
Blockly.loadWorkspace = function(workspace_loading_function) {
  Blockly.asyncCallWebService('robot_blockly', 'get_block_packages', '',
    function(data) {
      if ("EMPTY" == data) {
        workspace_loading_function();
      }
      else {
        var scriptsToLoad = data;
        var deferreds = [];
        var arrayLength = scriptsToLoad.length;
        for (var i = 0; i < arrayLength; i++) {
          deferreds.push($.getScript(scriptsToLoad[i]));
        }
        $.when.apply($, deferreds).then(function () {
          if (Blockly.waitForComponentsList.length > 0) {
            $.when.apply($, Blockly.waitForComponentsList).then(function () {
              workspace_loading_function();
            });
          }
          else {
            workspace_loading_function();
          }
        });
      }
    });
}
