package com.seulawah.tracking;

import java.sql.*;
import java.time.LocalTime;

public class JDBC {

    public final String url = "jdbc:postgresql://localhost/BelajarAIUEO";
    public final String user = "postgres";
    public final String password = "admin";

    public Connection connect() {
        Connection conn = null;
        try {
            conn = DriverManager.getConnection(url, user, password);
        } catch (SQLException e) {
            System.out.println(e.getMessage());
        }
        return conn;
    }

}
