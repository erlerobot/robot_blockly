
var ExecutionLogicModule = (function () {

  var CODE_STATUS = {
    RUNNING: "running",
    PAUSED: "paused",
    COMPLETED: "completed",
    NOT_CONNECTED: "not_connected"
  };

  var LAUNCH_BUTTON_ID = "launch_button";
  var current_status = CODE_STATUS.NOT_CONNECTED;
  var current_block = null;
  var socket = null;

  function update_launch_button() {
    var launch_button = document.getElementById(LAUNCH_BUTTON_ID);
    switch (current_status) {
      case CODE_STATUS.PAUSED:
        launch_button.innerHTML = "Resume";
        break;

      case CODE_STATUS.RUNNING:
        launch_button.innerHTML = "Pause";
        break;

      case CODE_STATUS.COMPLETED:
        launch_button.innerHTML = "Launch";
        break;

      case CODE_STATUS.NOT_CONNECTED:
        launch_button.innerHTML = "Server is down";
        break;

      default:
        console.log('Unknown current status: ' + current_status);
        break;
    }
  }

  function update_workspace() {
    var blocks_tab_selector = "a[href='#home'][data-toggle='tab']";
    var python_tab_selector = "a[href='#profile'][data-toggle='tab']";
    var graph_tab_selector = "a[href='graph.html']";
    var load_from_file_button_selector = "a[id='load_from_file_button']";
    var save_to_file_button_selector = "a[id='save_to_file_button']";
    switch (current_status) {
      case CODE_STATUS.PAUSED:
      case CODE_STATUS.RUNNING:
        workspace.options.readOnly = true;
        if (null != workspace.toolbox_) {
          workspace.toolbox_.HtmlDiv.hidden = true;
        }
        $(blocks_tab_selector).hide();
        $(python_tab_selector).hide();
        $(graph_tab_selector).hide();
        $(load_from_file_button_selector).hide();
        $(save_to_file_button_selector).hide();
        break;

      case CODE_STATUS.COMPLETED:
      case CODE_STATUS.NOT_CONNECTED:
        current_block = null;
        var blocks = workspace.getAllBlocks();
        for (var i = 0; i < blocks.length; i++) {
          blocks[i].setShadow(false);
        }
        workspace.options.readOnly = false;
        if (null != workspace.toolbox_) {
          workspace.toolbox_.HtmlDiv.hidden = false;
        }
        $(blocks_tab_selector).show();
        $(python_tab_selector).show();
        $(graph_tab_selector).show();
        $(load_from_file_button_selector).show();
        $(save_to_file_button_selector).show();
        break;

      default:
        console.log('Unknown current status: ' + current_status);
        break;
    }
  }

  function set_current_status(value) {
    var statuses = [CODE_STATUS.COMPLETED, CODE_STATUS.RUNNING, CODE_STATUS.PAUSED, CODE_STATUS.NOT_CONNECTED];
    if (0 > statuses.indexOf(value)) {
      console.log('Unknown status: ' + value);
      return;
    }
    current_status = value;
    update_launch_button();
    update_workspace();
  }

  function set_current_block_id(block_id) {
    if ([CODE_STATUS.RUNNING, CODE_STATUS.PAUSED].indexOf(current_status) >= 0) {
      var selected_block = workspace.getBlockById(block_id);
      if (null != selected_block) {
        if (null != current_block) {
          current_block.setShadow(false);
        }
        selected_block.setShadow(true);
        current_block = selected_block;
      } else {
        console.log('Not existing block id: ' + block_id);
      }
    } else {
      console.log('Code is not running. Ignoring current block changed event.')
    }
  }

  function is_connection_closed() {
    return (CODE_STATUS.NOT_CONNECTED == current_status);
  }

  return {

    launch_websockets: function () {
      var host_name = window.location.hostname;
      socket = new WebSocket("ws://" + host_name + ":9000");
      // socket = new WebSocket("ws://127.0.0.1:9000");
      // socket = new WebSocket("ws://10.0.0.1:9000");
      // socket = new WebSocket("ws:/erle-brain-2.local:9000");
      // socket = new WebSocket("ws://192.168.1.57:9000");
      socket.binaryType = "arraybuffer";

      socket.onopen = function () {
        console.log("Connected!");
        set_current_status(CODE_STATUS.COMPLETED);
      };

      socket.onmessage = function (e) {
        if (typeof e.data == "string") {
          console.log("Text message received: " + e.data);
          var message_data = e.data.split('\n');
          var method_name = '';
          var method_data = '';
          if (message_data.length > 0) {
            method_name = message_data[0];
            if (message_data.length > 1) {
              method_data = message_data[1];
            }
          }

          switch (method_name) {
            case 'set_current_block':
              set_current_block_id(method_data);
              if (CODE_STATUS.COMPLETED == current_status) {
                set_current_status(CODE_STATUS.RUNNING);
              }
              break;

            case 'status_update':
              set_current_status(method_data);
              break;

            default:
              console.log('Unknown method: ' + method_name);
              break;
          }
        } else {
          var arr = new Uint8Array(e.data);
          var hex = '';
          for (var i = 0; i < arr.length; i++) {
            hex += ('00' + arr[i].toString(16)).substr(-2);
          }
          console.log("Binary message received: " + hex);
        }
      };

      socket.onclose = function (e) {
        socket = null;
        set_current_status(CODE_STATUS.NOT_CONNECTED);
        console.log("Connection closed. Reason: " + e.reason);
      };
    },

    launch_code: function () {

      if (is_connection_closed()) {
        console.log("Connection not opened.");
        return;
      }
      var message_data = '';
      switch (current_status) {
        case CODE_STATUS.COMPLETED:
          message_data = 'play\n';
          Blockly.Python.addReservedWords('code');
          var saved_statement_prefix = Blockly.Python.STATEMENT_PREFIX;
          try {
            Blockly.Python.STATEMENT_PREFIX = 'check_status(%1)\n';
            var code = Blockly.Python.workspaceToCode(workspace);
            if (0 == code.length) {
              code = 'pass\n'
            }
            message_data += '\ntry:\n' + Blockly.Python.prefixLines(code, Blockly.Python.INDENT) + '\nfinally:\n' +
              Blockly.Python.INDENT + 'send_status_completed()\n';
          }
          finally {
            Blockly.Python.STATEMENT_PREFIX = saved_statement_prefix;
          }
          set_current_status(CODE_STATUS.RUNNING);
          break;

        case CODE_STATUS.RUNNING:
          set_current_status(CODE_STATUS.PAUSED);
          message_data = "pause";
          break;

        case CODE_STATUS.PAUSED:
          set_current_status(CODE_STATUS.RUNNING);
          message_data = "resume";
          break;

        default:
          console.log("Unknown status: " + current_status);
          break;
      }

      if (message_data.length > 0) {
        socket.send(message_data);
        console.log("Text message sent.");
      }
    },

    load_from_file: function() {
      var can_load_file = false;
      if (workspace.getAllBlocks().length > 0) {
        can_load_file = confirm("Current workspace is not empty. Do you want to override it?");
      } else {
        can_load_file = true;
      }

      if (true == can_load_file) {
        var input_field_name = 'load_workspace_from_file_input';
        var file_input = document.getElementById(input_field_name);
        if (null == file_input) {
          file_input = document.createElement('input');
          file_input.type = 'file';
          file_input.id = input_field_name;
          file_input.name = input_field_name;
          file_input.addEventListener('change',
            function (evt) {
              var files = evt.target.files;
              if (files.length > 0) {
                var file = files[0];
                var reader = new FileReader();
                reader.onload = function () {
                  workspace.clear();
                  var xml = Blockly.Xml.textToDom(this.result);
                  console.log("Loading workspace from file.");
                  Blockly.Xml.domToWorkspace(xml, workspace);
                };
                reader.readAsText(file);
                // This is done in order to allow open the same file several times in the row
                document.body.removeChild(file_input);
              }
            }, false);
          // Hidding element from view
          file_input.style = 'position: fixed; top: -100em';
          document.body.appendChild(file_input);
        }
        file_input.click();
      }
    },

    save_to_file: function() {
      var filename = 'blockly_workspace.xml';
      var xml = Blockly.Xml.workspaceToDom(workspace);
      var xml_text = Blockly.Xml.domToText(xml);
      var blob = new Blob([xml_text], {type: 'text/xml'});
      if (window.navigator.msSaveOrOpenBlob) {
        window.navigator.msSaveBlob(blob, filename);
      } else {
        var elem = window.document.createElement('a');
        elem.href = window.URL.createObjectURL(blob);
        elem.download = filename;
        document.body.appendChild(elem);
        elem.click();
        document.body.removeChild(elem);
      }
      console.log("Workspace saved.");
    },

    save_js_file: function() {
      $('<div id="save_function_dialog"></div>').dialog({
        modal: true,
        title: "Save function",
        open: function () {
          // Hidding close button on dialog title bar
          $(".ui-dialog-titlebar-close", $(this).parent()).hide();

          var functions = '';
          var allProcedures = Blockly.Procedures.allProcedures(workspace);
          var allProceduresLength = allProcedures.length;
          if (2 == allProceduresLength) {
            for (var i = 0; i < allProceduresLength; ++i) {
              var proceduresCategoryLength = allProcedures[i].length;
              for (var j = 0; j < proceduresCategoryLength; ++j) {
                var procedureName = allProcedures[i][j][0];
                functions += '<option value="' + procedureName + '">' + procedureName + '</option>';
              }
            }
            if (0 == functions.length) {
              $(this).html('There are no functions in the workspace to save.');
            }
            else {
              $(this).html('<form><fieldset>' +
                '<label for="category">Category</label><input type="text" name="category" id="category">' +
                '<label for="function">Function</label><select name="function" id="function">' + functions + '</select>' +
                '</fieldset></form>');
            }
          }
          else {
            $(this).html('Error during dialog initialization. Please read logs for more information.');
            console.log('Number of procedure types should be two (with and without return value). But it is ' +
              allProceduresLength);
          }
        },
        buttons: {
          Save: function () {
            $(this).dialog("close");
            $("#save_function_dialog").remove();
          },
          Cancel: function () {
            $(this).dialog("close");
            $("#save_function_dialog").remove();
          }
        }
      });
    }
  };
})();
