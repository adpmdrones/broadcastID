<?php

	include("connect.php");

	$link=Connection();

	$result=mysql_query("SELECT * FROM `esp32log` ORDER BY `timeStamp` DESC LIMIT 20",$link);
?>

<html>
   <head>
      <title>ESP32 Data</title>
   </head>
<body>
   <h1>Lat / Lon / BroadcastID sensor readings</h1>

   <table border="1" cellspacing="1" cellpadding="1">
		<tr>
			<td>&nbsp;Timestamp&nbsp;</td>
			<td>&nbsp;Lat&nbsp;</td>
			<td>&nbsp;Lon&nbsp;</td>
			<td>&nbsp;BroadcastID 1&nbsp;</td>
		</tr>

      <?php
		  if($result!==FALSE){
		     while($row = mysql_fetch_array($result)) {
		        printf("<tr><td> &nbsp;%s </td><td> &nbsp;%s&nbsp; </td><td> &nbsp;%s&nbsp; </td><td> &nbsp;%s&nbsp; </td></tr>",
		           $row["timestamp"], $row["lat"], $row["lon"], $row["broadcastid"]);
		     }
		     mysql_free_result($result);
		     mysql_close();
		  }
      ?>

   </table>
</body>
</html>


