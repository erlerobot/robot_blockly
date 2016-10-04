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

if (null == Blockly.toolboxXmlDocument) {
  Blockly.toolboxXmlDocument = $.parseXML('<xml xmlns="http://www.w3.org/1999/xhtml"></xml>');
}

/**
 * Append toolbox xml to already existing during initialization of the workspace.
 * @param {string} xml code of the toolbox to append to already existing one.
 */
Blockly.appendToToolbox = function(xml) {

  var mergeTwoNodes = function(originalDocumentNode, appendedDocumentNode) {
    for (var i = 0; i < appendedDocumentNode.childNodes.length; i++) {
      var appendedNode = appendedDocumentNode.childNodes[i];
      var foundCategory = false;
      if ('category' == appendedNode.nodeName) {
        for (var j = 0; !foundCategory && (j < originalDocumentNode.childNodes.length); j++) {
          var originalNode = originalDocumentNode.childNodes[j];
          if (('category' == originalNode.nodeName) &&
            (appendedNode.attributes['name'].nodeValue == originalNode.attributes['name'].nodeValue)) {

            mergeTwoNodes(originalNode, appendedNode);
            foundCategory = true;
          }
        }
      }
      if (!foundCategory) {
        originalDocumentNode.appendChild(appendedNode);
        i--;
      }
    }
  }

  var appendedXmlDocument = $.parseXML('<xml xmlns="http://www.w3.org/1999/xhtml">' + xml + '</xml>');
  mergeTwoNodes(Blockly.toolboxXmlDocument.documentElement, appendedXmlDocument.documentElement);
}

/**
 * Append missing functions declarations to workspace.
 * @param {string} workspaceXml XML string of the workspace.
 * @param {string} functionsXml XML document which contains all functions which need to be imported.
 * @return {object} it contains two fields with fixed workspace XML with injected functions and collapsed functions XML .
 */
Blockly.importFunctionsToWorkspace = function(workspaceXml, functionsXml) {

  if ((null != workspaceXml)) {
    workspaceXml = $.trim(workspaceXml);
  }

  if ((null == workspaceXml) || (0 == workspaceXml.length)) {
    workspaceXml = '<xml xmlns="http://www.w3.org/1999/xhtml"></xml>';
  }

  var xmlDocument = $.parseXML(functionsXml);
  var $xmlDocument = $(xmlDocument);
  var proceduresList = $('block', $xmlDocument).filter(
    '[type="procedures_defnoreturn"],[type="procedures_defreturn"]');
  for (var i = 0; i < proceduresList.length; i++) {
    var collapsedAttribute = proceduresList[i].attributes['collapsed'];
    if (null == collapsedAttribute) {
      proceduresList[i].setAttribute('collapsed', 'true')
    }
    var nameSearchResult = $('field[name="NAME"]', proceduresList[i]);
    if (0 == nameSearchResult.length) {
      console.log('Could not find name for procedure. Skipping it.')
    }
    else {
      var procedureName = nameSearchResult[0].innerText;
      if (-1 == workspaceXml.indexOf('>' + procedureName + '</field>')) {
        workspaceXml = workspaceXml.replace('</xml>', '') + Blockly.Xml.domToText(proceduresList[i]) +
          '</xml>';
      }
    }
  }

  var fixedFunctionXml = '';
  if (null != xmlDocument.documentElement.firstChild) {
    fixedFunctionXml = Blockly.Xml.domToText(xmlDocument.documentElement.firstChild);
  }

  return {
    workspaceXml: workspaceXml,
    functionsXml: fixedFunctionXml
  };
}

/**
 * Append toolbox xml to category during initialization of the workspace.
 * @param {string} categoryName full name of the category, is separated by / sign.
 * @param {string} xml code of the toolbox to append to already existing one.
 */
Blockly.appendToToolboxCategory = function(categoryName, xml) {
  if (null != categoryName) {
    var categoriesList = categoryName.split('/');
    var xmlText = xml;
    for (var i = categoriesList.length - 1; i >= 0; i--) {
      xmlText = '<category name="' + categoriesList[i] + '">' + xmlText + '</category>';
    }

    var cachedWorkspaceXml = localStorage.getItem("blocks_cache");
    var importResult = Blockly.importFunctionsToWorkspace(cachedWorkspaceXml,
      '<xml xmlns="http://www.w3.org/1999/xhtml">' + xmlText + '</xml>');

    localStorage.setItem("blocks_cache", importResult.workspaceXml);
    Blockly.appendToToolbox(importResult.functionsXml);
  }
}

/**
 * Return toolbox XML during initialization of the workspace.
 */
Blockly.getToolboxXmlText = function() {
  return Blockly.Xml.domToText(Blockly.toolboxXmlDocument);
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
