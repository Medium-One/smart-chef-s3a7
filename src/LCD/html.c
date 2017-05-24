/***********************************************************************************************************************
 * Copyright [2015] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
 *
 * The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
 * and/or its licensors ("Renesas") and subject to statutory and contractual protections.
 *
 * Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
 * display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
 * purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
 * SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 * NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
 * INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
 * OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
 * be subject to different terms.
 **********************************************************************************************************************/

 /*******************************************************************************
 * File Name    : html.c
 * Version      : 1.0
 * Device(s)    : S3A7
 * Tool-Chain   : e2studio, GNU GCC 4.9
 * OS           : ThreadX
 * H/W Platform : S3A7 IoT Enabler
 * Description  : Provisioning HTML page contents, stored to QSPI.
 ******************************************************************************/

#include "hal_data.h"

const char provisionHtml[] BSP_PLACE_IN_SECTION(".qspi_flash") =
        "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\"><title>Kit Configuration</title><meta nam"
        "e=\"viewport\" content=\"width=device-width,initial-scale=1\"><style>#content,#footer{max-width:660px;ma"
        "rgin:0 auto}#content,#footer,body{margin:0 auto}*{-webkit-box-sizing:border-box;-moz-box-sizing:bord"
        "er-box;box-sizing:border-box}body{font-family:Verdana,Arial,sans-serif;padding:0 0 20px}#page{backgr"
        "ound:#fff;padding:20px;font-size:11pt}#page td{font-size:10pt}#title{font-weight:700;color:#000;marg"
        "in:20px 0;text-align:center}#title .left{float:left;text-align:left}#footer{height:16px;padding:3px "
        "5px 20px;color:#888;font-size:.9em;text-align: center}#content a:hover{color:#d33;text-decoration:un"
        "derline}.rounded_container{display:block;position:relative;border:2px solid #000;-webkit-border-radi"
        "us:15px;-webkit-user-select:none;-moz-user-select:none;-ms-user-select:none;user-select:none;font-si"
        "ze:12pt;-webkit-transition:height .5s;-moz-transition:height .5s;transition:height .5s;padding:12px "
        "0 0}.rounded_container table tr{width:9em}.text-right{text-align:right}.join_button{text-indent:5;fo"
        "nt-family:Arial;font-style:normal;text-align:center;width:100px;height:30px;margin:10px 0 5px}.ssid1"
        "{width:213px}.logo{display:-webkit-box;display:-webkit-flex;display:-ms-flexbox;display:-moz-box;dis"
        "play:flex;-webkit-box-pack:justify;-webkit-justify-content:space-between;-ms-flex-pack:justify;-moz-"
        "box-pack:justify;justify-content:space-between;-webkit-box-align:center;-webkit-align-items:center;-"
        "ms-flex-align:center;-moz-box-align:center;align-items:center;margin-left:auto;margin-right:auto}img"
        "{display:block;height:auto;max-width:100%}@media (max-width:660px){.logo{display:block;text-align:ce"
        "nter;margin-left:auto;margin-right:auto}.logo img{margin:10px auto}.rounded_container table{width:10"
        "0%}.rounded_container table tr{width:auto}.ssid1{width:100%}}table{margin-left:auto;margin-right:aut"
        "o}.grey_text{color:#888;font-size:.8em}.align_right{text-align: right}.rounded_container iframe{bord"
        "er:0;overflow:hidden;width:100%}.hidden{display:none;}</style><script>function joinNetwork(){var ssi"
        "d=document.getElementById('s').value;if(ssid.length<=0){alert('Please enter a valid SSID.');return;}"
        "document.getElementById(\"ssid\").value=ssid;var sec=document.getElementById('am').value;document.getE"
        "lementById(\"sec\").value=sec;var key=document.getElementById('k').value;document.getElementById(\"key\""
        ").value=key;var mqttprojectid=document.getElementById('m').value;if(mqttprojectid.length<=0){alert('"
        "Please enter a valid mqtt project id.');return;}document.getElementById(\"mqttprojectid\").value=mqttp"
        "rojectid;var mqttuserid=document.getElementById('u').value;if(mqttuserid.length<=0){alert('Please en"
        "ter a valid user_mqtt_id.');return;}document.getElementById(\"mqttuserid\").value=mqttuserid;var apike"
        "y=document.getElementById('a').value;if(apikey.length<=0){alert('Please enter a valid api_key.');ret"
        "urn;}document.getElementById(\"apikey\").value=apikey;var password=document.getElementById('p').value;"
        "if(password.length<=0){alert('Please enter a valid api_password.');return;}document.getElementById(\""
        "password\").value=password;window.location.hash='s='+ssid+'&am='+sec+'&m='+mqttprojectid+'&u='+mqttus"
        "erid+'&a='+apikey+'&p='+password;document.kickoff.submit();}function fillInputHashParams(){var hashP"
        "arams=window.location.hash.substr(1).split('&');for(var i=0;i<hashParams.length;i++){var p=hashParam"
        "s[i].split('=');if(p.length==1){break;}document.getElementById(p[0]).value=decodeURIComponent(p[1]);"
        "}}window.onload=fillInputHashParams;window.onhashchange=fillInputHashParams;</script></head><body><d"
        "iv id=\"page\"><div id=\"content\"><img class=\"logo\" src=\"data:image/jpeg;base64,/9j/4AAQSkZJRgABAQAASAB"
        "IAAD/4QCMRXhpZgAATU0AKgAAAAgABQESAAMAAAABAAEAAAEaAAUAAAABAAAASgEbAAUAAAABAAAAUgEoAAMAAAABAAIAAIdpAAQ"
        "AAAABAAAAWgAAAAAAAABIAAAAAQAAAEgAAAABAAOgAQADAAAAAQABAACgAgAEAAAAAQAAAMegAwAEAAAAAQAAAB0AAAAA/+0AOFB"
        "ob3Rvc2hvcCAzLjAAOEJJTQQEAAAAAAAAOEJJTQQlAAAAAAAQ1B2M2Y8AsgTpgAmY7PhCfv/CABEIAB0AxwMBEQACEQEDEQH/xAA"
        "fAAABBQEBAQEBAQAAAAAAAAADAgQBBQAGBwgJCgv/xADDEAABAwMCBAMEBgQHBgQIBnMBAgADEQQSIQUxEyIQBkFRMhRhcSMHgSC"
        "RQhWhUjOxJGIwFsFy0UOSNIII4VNAJWMXNfCTc6JQRLKD8SZUNmSUdMJg0oSjGHDiJ0U3ZbNVdaSVw4Xy00Z2gONHVma0CQoZGig"
        "pKjg5OkhJSldYWVpnaGlqd3h5eoaHiImKkJaXmJmaoKWmp6ipqrC1tre4ubrAxMXGx8jJytDU1dbX2Nna4OTl5ufo6erz9PX29/j"
        "5+v/EAB8BAAMBAQEBAQEBAQEAAAAAAAECAAMEBQYHCAkKC//EAMMRAAICAQMDAwIDBQIFAgQEhwEAAhEDEBIhBCAxQRMFMCIyURR"
        "ABjMjYUIVcVI0gVAkkaFDsRYHYjVT8NElYMFE4XLxF4JjNnAmRVSSJ6LSCAkKGBkaKCkqNzg5OkZHSElKVVZXWFlaZGVmZ2hpanN"
        "0dXZ3eHl6gIOEhYaHiImKkJOUlZaXmJmaoKOkpaanqKmqsLKztLW2t7i5usDCw8TFxsfIycrQ09TV1tfY2drg4uPk5ebn6Onq8vP"
        "09fb3+Pn6/9sAQwAQCwwODAoQDg0OEhEQExgoGhgWFhgxIyUdKDozPTw5Mzg3QEhcTkBEV0U3OFBtUVdfYmdoZz5NcXlwZHhcZWd"
        "j/9sAQwEREhIYFRgvGhovY0I4QmNjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2Nj/9oADAM"
        "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\" alt=\""
        "Medium One\"><div id=\"title\"><h2> Renesas S3A7 Kit Configuration </h2><div> Visit <a href=\"http://med"
        "iumone.com/renesas\">mediumone.com/renesas</a> for activation instructions and to learn more </div></"
        "div><div class=\"rounded_container\" id=\"network_config\"><table><tr>  <td></td>  <td></td></tr><tr>  <"
        "td class=\"align_right\">Wifi Network SSID:</td>  <td>  <input id=\"s\" type=\"text\" name=\"m\" maxlength=\""
        "255\" class=\"ssid1\" value=\"\">  </td></tr><tr>  <td class=\"align_right\">Authentication Mode:</td>  <td"
        ">  <select id=\"am\" name=\"am\">  <option value=\"open\">Open</option>  <option value=\"open\">Open</option"
        ">  <option value=\"wep40bit\">WEP 40-bit Open Hex Key</option>  <option value=\"wep128bit\">WEP 128-bit "
        "Open Hex Key</option>  <option value=\"wpa\">WPA</option>  <option value=\"wpa2\" selected=\"\">WPA2</opti"
        "on>  </select>  </td></tr><tr>  <td class=\"align_right\">WiFI Password:</td>  <td>  <input id=\"k\" typ"
        "e=\"text\" name=\"k\" maxlength=\"255\" class=\"ssid1\" value=\"\">  </td></tr><tr>  <td class=\"align_right\">P"
        "roject MQTT ID:</td>  <td>  <input id=\"m\" type=\"text\" name=\"m\" maxlength=\"255\" class=\"ssid1\" value=\""
        "\">  </td></tr><tr>  <td class=\"align_right\">User MQTT ID:</td>  <td>  <input id=\"u\" type=\"text\" name"
        "=\"u\" maxlength=\"255\" class=\"ssid1\" value=\"\">  </td></tr><tr>  <td class=\"align_right\">API Key:</td> "
        " <td>  <input id=\"a\" type=\"text\" name=\"a\" maxlength=\"255\" class=\"ssid1\" value=\"\">  </td></tr><tr>  <"
        "td class=\"align_right\">API User Password:</td>  <td>  <input id=\"p\" type=\"text\" name=\"p\" maxlength=\""
        "255\" class=\"ssid1\" value=\"\">  </td></tr><tr>  <td colspan=\"2\" class=\"text-right\">  <input id=\"joinBu"
        "tton\" class=\"join_button\" type=\"button\" value=\"Connect\"  onclick=\"joinNetwork();\">  </td></tr></tabl"
        "e></div><br><form method=\"post\" name=\"kickoff\" action=\"\"><input type=\"hidden\" id=\"ssid\" name=\"ssid\"/"
        "><input type=\"hidden\" id=\"sec\" name=\"sec\"/><input type=\"hidden\" id=\"key\" name=\"key\"/><input type=\"hi"
        "dden\" id=\"mqttprojectid\" name=\"mqttprojectid\"/><input type=\"hidden\" id=\"mqttuserid\" name=\"mqttuserid"
        "\"/><input type=\"hidden\" id=\"apikey\" name=\"apikey\"/><input type=\"hidden\" id=\"password\" name=\"password"
        "\"/></form></div><br><div id=\"footer\"> Copyright &copy; 2016, Medium One Inc. </div></div></body></ht"
        "ml>"
        ;
