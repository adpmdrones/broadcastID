<?php
   	include("connect.php");

   	$link=Connection();

	$lat=$_GET["lat"];
	$lon=$_GET["lat"];
	$broadcastid=$_GET["broadcastid"];

	$query = "INSERT INTO `esp32log` (`lat`, `lon`, `broadcastid`)
		VALUES ('".$lat."','".$lon."','".$broadcastid."')";

   	mysql_query($query,$link);
	mysql_close($link);

   	header("Location: index.php");
?>
