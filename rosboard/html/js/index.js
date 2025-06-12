"use strict";

importJsOnce("js/viewers/meta/Viewer.js");
importJsOnce("js/viewers/meta/Space2DViewer.js");
importJsOnce("js/viewers/meta/Space3DViewer.js");

importJsOnce("js/viewers/ImageViewer.js");
importJsOnce("js/viewers/LogViewer.js");
importJsOnce("js/viewers/ProcessListViewer.js");
importJsOnce("js/viewers/MapViewer.js");
importJsOnce("js/viewers/LaserScanViewer.js");
importJsOnce("js/viewers/GeometryViewer.js");
importJsOnce("js/viewers/PolygonViewer.js");
importJsOnce("js/viewers/DiagnosticViewer.js");
importJsOnce("js/viewers/TimeSeriesPlotViewer.js");
importJsOnce("js/viewers/PointCloud2Viewer.js");

// GenericViewer must be last
importJsOnce("js/viewers/GenericViewer.js");

importJsOnce("js/transports/WebSocketV1Transport.js");

var snackbarContainer = document.querySelector('#demo-toast-example');

let subscriptions = {};

if(window.localStorage && window.localStorage.subscriptions) {
  if(window.location.search && window.location.search.indexOf("reset") !== -1) {
    subscriptions = {};
    updateStoredSubscriptions();
    window.location.href = "?";
  } else {
    try {
      subscriptions = JSON.parse(window.localStorage.subscriptions);
    } catch(e) {
      console.log(e);
      subscriptions = {};
    }
  }
}

let $grid = null;
$(() => {
  $grid = $('.grid').masonry({
    itemSelector: '.card',
    gutter: 10,
    percentPosition: true,
  });
  $grid.masonry("layout");

  // Initialize control panel button handlers
  initControlPanel();

  // Initialize Robot Info panel with placeholder values
  updateRobotInfo({
    mode: "Placeholder",
    currentLocation: "Placeholder",
    targetLocation: "Placeholder",
    forksStatus: "Placeholder",
    taskName: "Placeholder",
    taskStatus: "Placeholder"
  });
});

setInterval(() => {
  if(currentTransport && !currentTransport.isConnected()) {
    console.log("attempting to reconnect ...");
    currentTransport.connect();
  }
}, 5000);

function updateStoredSubscriptions() {
  if(window.localStorage) {
    let storedSubscriptions = {};
    for(let topicName in subscriptions) {
      storedSubscriptions[topicName] = {
        topicType: subscriptions[topicName].topicType,
      };
    }
    window.localStorage['subscriptions'] = JSON.stringify(storedSubscriptions);
  }
}

function newCard() {
  let card = $("<div></div>").addClass('card')
    .appendTo($('.grid'));
  return card;
}

let onOpen = function() {
  for(let topic_name in subscriptions) {
    console.log("Re-subscribing to " + topic_name);
    initSubscribe({topicName: topic_name, topicType: subscriptions[topic_name].topicType});
  }
}

let onSystem = function(system) {
  if(system.hostname) {
    console.log("hostname: " + system.hostname);
    $('.mdl-layout-title').text("ROSboard: " + system.hostname);
  }

  if(system.version) {
    console.log("server version: " + system.version);
    versionCheck(system.version);
  }
}

let onMsg = function(msg) {
  if(!subscriptions[msg._topic_name]) {
    console.log("Received unsolicited message", msg);
  } else if(!subscriptions[msg._topic_name].viewer) {
    console.log("Received msg but no viewer", msg);
  } else {
    subscriptions[msg._topic_name].viewer.update(msg);
  }
}

let currentTopics = {};
let currentTopicsStr = "";

let onTopics = function(topics) {
  let newTopicsStr = JSON.stringify(topics);
  if(newTopicsStr === currentTopicsStr) return;
  currentTopics = topics;
  currentTopicsStr = newTopicsStr;
  
  let topicTree = treeifyPaths(Object.keys(topics));
  
  $("#topics-nav-ros").empty();
  $("#topics-nav-system").empty();
  
  addTopicTreeToNav(topicTree[0], $('#topics-nav-ros'));

  $('<a></a>')
  .addClass("mdl-navigation__link")
  .click(() => { initSubscribe({topicName: "_dmesg", topicType: "rcl_interfaces/msg/Log"}); })
  .text("dmesg")
  .appendTo($("#topics-nav-system"));

  $('<a></a>')
  .addClass("mdl-navigation__link")
  .click(() => { initSubscribe({topicName: "_top", topicType: "rosboard_msgs/msg/ProcessList"}); })
  .text("Processes")
  .appendTo($("#topics-nav-system"));

  $('<a></a>')
  .addClass("mdl-navigation__link")
  .click(() => { initSubscribe({topicName: "_system_stats", topicType: "rosboard_msgs/msg/SystemStats"}); })
  .text("System stats")
  .appendTo($("#topics-nav-system"));
}

function addTopicTreeToNav(topicTree, el, level = 0, path = "") {
  topicTree.children.sort((a, b) => {
    if(a.name>b.name) return 1;
    if(a.name<b.name) return -1;
    return 0;
  });
  topicTree.children.forEach((subTree, i) => {
    let subEl = $('<div></div>')
    .css(level < 1 ? {} : {
      "padding-left": "0pt",
      "margin-left": "12pt",
      "border-left": "1px dashed #808080",
    })
    .appendTo(el);
    let fullTopicName = path + "/" + subTree.name;
    let topicType = currentTopics[fullTopicName];
    if(topicType) {
      $('<a></a>')
        .addClass("mdl-navigation__link")
        .css({
          "padding-left": "12pt",
          "margin-left": 0,
        })
        .click(() => { initSubscribe({topicName: fullTopicName, topicType: topicType}); })
        .text(subTree.name)
        .appendTo(subEl);
    } else {
      $('<a></a>')
      .addClass("mdl-navigation__link")
      .attr("disabled", "disabled")
      .css({
        "padding-left": "12pt",
        "margin-left": 0,
        opacity: 0.5,
      })
      .text(subTree.name)
      .appendTo(subEl);
    }
    addTopicTreeToNav(subTree, subEl, level + 1, path + "/" + subTree.name);
  });
}

function initSubscribe({topicName, topicType}) {
    if(!subscriptions[topicName]) {
        subscriptions[topicName] = {
            topicType: topicType,
        }
    }  
    currentTransport.subscribe({topicName: topicName});
    if(!subscriptions[topicName].viewer) {
        let card = newCard();
        let viewer = Viewer.getDefaultViewerForType(topicType);
        try {
            subscriptions[topicName].viewer = new viewer(card, topicName, topicType);
            if (viewer === ImageViewer) {
                card.addClass('image-viewer-card');
            } else if (viewer === MapViewer) {
                card.addClass('map-viewer-card');
            }
        } catch(e) {
            console.error(e);
            card.remove();
        }
        $grid.masonry("appended", card);
        $grid.masonry("layout");
    }
    updateStoredSubscriptions();
}

let currentTransport = null;

function initDefaultTransport() {
  currentTransport = new WebSocketV1Transport({
    path: "/rosboard/v1",
    onOpen: onOpen,
    onMsg: onMsg,
    onTopics: onTopics,
    onSystem: onSystem,
  });
  currentTransport.connect();
}

function treeifyPaths(paths) {
  let result = [];
  let level = {result};

  paths.forEach(path => {
    path.split('/').reduce((r, name, i, a) => {
      if(!r[name]) {
        r[name] = {result: []};
        r.result.push({name, children: r[name].result})
      }
      
      return r[name];
    }, level)
  });
  return result;
}

let lastBotherTime = 0.0;
function versionCheck(currentVersionText) {
  $.get("https://raw.githubusercontent.com/dheera/rosboard/release/setup.py").done((data) => {
    let matches = data.match(/version='(.*)'/);
    if(matches.length < 2) return;
    let latestVersion = matches[1].split(".").map(num => parseInt(num, 10));
    let currentVersion = currentVersionText.split(".").map(num => parseInt(num, 10));
    let latestVersionInt = latestVersion[0] * 1000000 + latestVersion[1] * 1000 + latestVersion[2];
    let currentVersionInt = currentVersion[0] * 1000000 + currentVersion[1] * 1000 + currentVersion[2];
    if(currentVersionInt < latestVersionInt && Date.now() - lastBotherTime > 1800000) {
      lastBotherTime = Date.now();
      snackbarContainer.MaterialSnackbar.showSnackbar({
        message: "New version of ROSboard available (" + currentVersionText + " -> " + matches[1] + ").",
        actionText: "Check it out",
        actionHandler: () => {window.location.href="https://github.com/dheera/rosboard/"},
      });
    }
  });
}

$(() => {
  if(window.location.href.indexOf("rosboard.com") === -1) {
    initDefaultTransport();
  }
});

Viewer.onClose = function(viewerInstance) {
  let topicName = viewerInstance.topicName;
  let topicType = viewerInstance.topicType;
  currentTransport.unsubscribe({topicName:topicName});
  $grid.masonry("remove", viewerInstance.card);
  $grid.masonry("layout");
  delete(subscriptions[topicName].viewer);
  delete(subscriptions[topicName]);
  updateStoredSubscriptions();
}

Viewer.onSwitchViewer = (viewerInstance, newViewerType) => {
  let topicName = viewerInstance.topicName;
  let topicType = viewerInstance.topicType;
  if(!subscriptions[topicName].viewer === viewerInstance) console.error("viewerInstance does not match subscribed instance");
  let card = subscriptions[topicName].viewer.card;
  subscriptions[topicName].viewer.destroy();
  delete(subscriptions[topicName].viewer);
  subscriptions[topicName].viewer = new newViewerType(card, topicName, topicType);
};

// Control panel button handlers
function initControlPanel() {
  const serverUrl = 'http://localhost:3000';
  const showSnackbar = (message, isError = false) => {
    snackbarContainer.MaterialSnackbar.showSnackbar({
      message: message,
      timeout: isError ? 5000 : 3000,
    });
  };

  const handleButtonClick = async (endpoint, data = null, buttonId) => {
    const button = $(`#${buttonId}`);
    button.prop('disabled', true);
    try {
      const response = await fetch(`${serverUrl}${endpoint}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: data ? JSON.stringify(data) : undefined,
      });
      const result = await response.json();
      if (result.success) {
        showSnackbar(result.message || 'Action completed');
      } else {
        showSnackbar(`Error: ${result.error}`, true);
      }
    } catch (error) {
      console.error(`Error calling ${endpoint}:`, error);
      showSnackbar(`Error: ${error.message}`, true);
    } finally {
      button.prop('disabled', false);
    }
  };

  $('#turn-on-button').click(() => handleButtonClick('/turn-on', null, 'turn-on-button'));
  $('#turn-off-button').click(() => handleButtonClick('/turn-off', null, 'turn-off-button'));
  $('#start-mapping-button').click(() => handleButtonClick('/start-mapping', null, 'start-mapping-button'));
  $('#stop-mapping-button').click(() => handleButtonClick('/stop-mapping', null, 'stop-mapping-button'));
  $('#save-map-button').click(() => {
    const mapName = $('#save-map-input').val().trim();
    if (!mapName) {
      showSnackbar('Please enter a map name', true);
      return;
    }
    handleButtonClick('/save-map', { mapName }, 'save-map-button');
  });
  $('#start-navigation-button').click(() => handleButtonClick('/start-navigation', null, 'start-navigation-button'));
  $('#stop-navigation-button').click(() => handleButtonClick('/stop-navigation', null, 'stop-navigation-button'));
  $('#load-map-button').click(() => {
    const mapName = $('#load-map-input').val().trim();
    if (!mapName) {
      showSnackbar('Please enter a map name', true);
      return;
    }
    handleButtonClick('/load-map', { mapName }, 'load-map-button');
  });

  // Fork control buttons
  $('#full-lift-button').click(async () => {
    const button = $('#full-lift-button');
    button.prop('disabled', true);
    try {
      const result = await window.control.fullLift();
      showSnackbar(result.message || 'Forks lifted', !result.success);
    } catch (error) {
      console.error('Error lifting forks:', error);
      showSnackbar(`Error: ${error.message}`, true);
    } finally {
      button.prop('disabled', false);
    }
  });

  $('#full-lower-button').click(async () => {
    const button = $('#full-lower-button');
    button.prop('disabled', true);
    try {
      const result = await window.control.fullLower();
      showSnackbar(result.message || 'Forks lowered', !result.success);
    } catch (error) {
      console.error('Error lowering forks:', error);
      showSnackbar(`Error: ${error.message}`, true);
    } finally {
      button.prop('disabled', false);
    }
  });
}

// Function to update Robot Info panel
function updateRobotInfo(info) {
  const panel = $('.robot-info-panel');
  panel.find('p').eq(0).html(`<strong>Mode:</strong> ${info.mode}`);
  panel.find('p').eq(1).html(`<strong>Current Location:</strong> ${info.currentLocation}`);
  panel.find('p').eq(2).html(`<strong>Target Location:</strong> ${info.targetLocation}`);
  panel.find('p').eq(3).html(`<strong>Forks Status:</strong> ${info.forksStatus}`);
  panel.find('p').eq(4).html(`<strong>Task Name:</strong> ${info.taskName}`);
  panel.find('p').eq(5).html(`<strong>Task Status:</strong> ${info.taskStatus}`);
}