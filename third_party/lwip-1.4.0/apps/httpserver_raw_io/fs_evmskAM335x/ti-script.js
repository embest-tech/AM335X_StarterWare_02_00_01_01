/*
 * demo.js
 *
 * Main script controlling the web-based demo.
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Default animation speed
 */
var speed = "fast";

/*
 * Default tool-tip for functions not currently supported.
 */
var tipNa  = "This function is not available.";

/*
 * Initialize info foils
 */
var InfoFoils = new Array();
var InfoIndex;

InfoFoils[0] =
  "This is page 1. Line 1.<br />" +
  "This is page 1. Line 2.<br />";

InfoFoils[1] =
  "This is page 2. Line 1.<br />" +
  "This is page 2. Line 2.<br />";

InfoFoils[2] =
  "This is page 3. Line 1.<br />" +
  "This is page 3. Line 2.<br />";

/*
 * Splash text
 */
var SplashText =
  "<h2>AM335x StarterWare Demo</h2>" +
  "<p>This webpage is being served by the webserver running on " +
  "the AM335x platform.</p>" +
  "<p>Click on the icons to change the demo on target";

var SplashTime = 3;

/*
 * Splash screen
 */
function CreateSplash()
{
  var pos  = $("#demo").position();
  var elem = $("#splash");

  elem.hide();
  elem.css("top", pos.top);
  elem.css("left", pos.left);
  elem.html('<div>' + SplashText + '</div>');

  $("#count").html('5');

  $('#splash').fadeIn("fast").delay(5000).fadeOut("fast");
}

/*
 * Information panel for showing dynamic content
 */
var PanelReady = 0;

/* Panel header */
function PanelHdr()
{
  $("#p_hdr").html('<a href="#" onclick="HidePanel();"><img src="up.png" /></a>');
}

/* Panel footer */
function PanelFtr()
{
  var txt;

  var p = '<a class="prev" href="#" onclick="GoBack();"><img src="prev.png" /></a>';
  var n = '<a class="next" href="#" onclick="GoNext();"><img src="next.png" /></a>'

  if (InfoIndex == 0) {
    txt = '';
  } else {
    txt = p;
  }

  if (InfoIndex < (InfoFoils.length - 1)) {
    txt += n;
  }

  $("#p_ftr").html(txt);
}

/* Panel content */
function PanelTxt()
{
  $("#p_txt").html(InfoFoils[InfoIndex]);
}

/* Show panel */
function ShowPanel(arg)
{
  if (PanelReady == 0) {
    var pos   = $("#demo").position();
    var panel = $('<div><div id="p_hdr"></div><div id="p_txt"></div><div id="p_ftr"></div></div>');

    panel.css("top", pos.top);
    panel.css("left", pos.left);
    panel.hide();
    panel.attr("id","panel").appendTo("body");

    PanelHdr();

    InfoIndex = 0;
    PanelReady = 1;
  }

  PanelTxt();
  PanelFtr();

  $("#panel").fadeIn(speed);
};

/* Goto next panel */
function GoNext()
{
  if (InfoIndex < (InfoFoils.length - 1))
    InfoIndex++;

  $("#p_txt").fadeOut(speed);
  PanelTxt();
  $("#p_txt").fadeIn(speed);

  if ((InfoIndex == 1) || (InfoIndex == InfoFoils.length - 1)) {
    $("#p_ftr").hide();
    PanelFtr();
    $("#p_ftr").show();
  }
};

/* Goto previous panel */
function GoBack()
{
  if (InfoIndex > 0)
    InfoIndex--;

  $("#p_txt").fadeOut(speed);
  PanelTxt();
  $("#p_txt").fadeIn(speed);

  if ((InfoIndex == 0) || (InfoIndex == InfoFoils.length - 2)) {
    $("#p_ftr").hide();
    PanelFtr();
    $("#p_ftr").show();
  }
};

/* Hide panel */
function HidePanel()
{
  $("#panel").fadeOut(speed);
};

/* Show tip associated with current icon */
function ShowTip(idx)
{
  var t = $("#tip");

  t.html(Icons[idx].tip);
  t.show();
}

/* Show tip associated with current icon */
function HideTip()
{
  var t = $("#tip");

  t.hide();
  t.html("");
}

/*
 * Create icons
 */
function CreateUI()
{
  var demo = $("#demo");

  var str = '';

  str += '<form id="ui" action="/io_control.cgi">' ;
  str += '<table id="icons"><tbody>' ;

  var col = 0;

  for (var i = 0; i < Icons.length; i++) {
    if (col++ == 0)
      str += '<tr>';

    str += '<td>';
    str += '<input type="submit" ';

    if ((typeof Unused != "undefined") &&
        (Unused instanceof Array) &&
        (jQuery.inArray(Icons[i].id, Unused) > -1)) {
      str += 'class="inactive" ';
    } else {
      str += 'class="active" ';
    }

    str += 'id="' + Icons[i].id + '" ';
    str += 'name="Update" ';
    str += 'value="' + Icons[i].txt + '" ';
    str += 'onmouseover="ShowTip(' + i + ');" ';
    str += 'onmouseout="HideTip();"';
    str += '/>';
    str += '</td>';

    if (col == 4) {
      str += '</tr>';
      col = 0;
    }
  }

  if ((col > 0) && (col < 4)) {
    while (col < 4) {
      str += '<td>&nbsp;</td>';
      col++;
    }
      str += '</tr>';
  }

  str += '</tbody></table>';
  str += '</form>' ;

  demo.html(str);
}

/*
 * Dim unused icons
 */
function NotUsed()
{
}

/* Dim an element and clear its link */
function Dim(id)
{
  var e = $(id);

/*  e.attr("class", "dim"); */
}

/* Dim unused icons */
function DimUnused()
{
  if ((typeof Unused != "undefined") && (Unused instanceof Array)) {
    for (var i in Unused)
    {
      var done = 0;
      var j = 0;

      while ((!done) && (j < Icons.length)) {
	if (Icons[j].id == Unused[i]) {
	  Icons[j].act = 'func:NotUsed()';
	  Icons[j].tip = tipNa ;
	  done = 1;
	}
	j++;
      } /* while */
    } /* for */
  }
}

/* Id of the icon clicked. (Initialize with invalid index) */
var ClickId = -1;

/* Register default handlers */
function RegEventHandlers()
{
  /* Handle event generated for "click" on the icons */
  $("#ui input").click(function(e) {
    var done = 0;
    var idx  = 0;

    /* Find the index of corresponding icon in Icons[] */
    while ((!done) && (idx < Icons.length)) {
      if (Icons[idx].id == this.id) {
	ClickId = idx;
	done = 1;
      } else {
	idx++;
      }
    }
  });

  /* Handle event generated for "submit" action */
  $("#ui").submit(function(e) {
    /* prevent default submit action */
    e.preventDefault();

    var act = Icons[ClickId].act;

    var r_func   = /^func:(.*)$/;
    var r_submit = /^submit:(.*)$/;

    if (r_func.test(act)) {
      act = act.replace(/func:/, "");
      eval(act);
    }
    else if (r_submit.test(act)) {
      var $form = $(this);
      var url   = $form.attr('action');

      act = act.replace(/submit:/, "");

      /* Submit form data as GET request. Ignore result. */
      $.get(url, { Update : act } );
    } else {
      alert ("No action specified");
    }

    ClickId  = -1;
  });
}

/*
 * Initialize content when the page is fully loaded
 */
$(document).ready(function() {
  $("#splash").hide();
  $("#tip").hide();
  CreateSplash();
  CreateUI();
  DimUnused();
  RegEventHandlers();
});
