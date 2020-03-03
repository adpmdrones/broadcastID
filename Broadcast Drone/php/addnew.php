<?php
   	include("connect.php");

   	$link=Connection();

	$lat=$_POST["lat"];
	$lon=$_POST["lon"];
	$broadcastid=$_POST["broadcastid"];

	$query = "INSERT INTO `esp32log` (`lat`, `lon`, `broadcastid`)
		VALUES ('".$lat."','".$lon."','".$broadcastid."')";

   	mysql_query($query,$link);
	mysql_close($link);

   	header("Location: index.php");
?>
