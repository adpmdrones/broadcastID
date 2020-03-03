<?php

	function Connection(){
		$server="localhost";
		$user="zu1lachr_adpm";
		$pass="2020adpm2020";
		$db="zu1lachr_test";

		$connection = mysql_connect($server, $user, $pass);

		if (!$connection) {
	    	die('MySQL ERROR: ' . mysql_error());
		}

		mysql_select_db($db) or die( 'MySQL ERROR: '. mysql_error() );

		return $connection;
	}
?>
