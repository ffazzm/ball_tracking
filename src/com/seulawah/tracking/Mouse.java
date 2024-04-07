package com.seulawah.tracking;

import javax.swing.*;
import java.awt.event.*;

import org.opencv.core.Point;

public class Mouse extends MouseAdapter {
    public int koX, koY;
    public boolean LeftMouseButton, RightMouseButton;
    public Point pt;

    /*&& event.getClickCount() == 2 && !event.isConsumed()
            event.consume();*/

    @Override
    public void mousePressed(MouseEvent event) {
        if (SwingUtilities.isLeftMouseButton(event)) {

            this.koX = event.getX();
            this.koY = event.getY();
            this.LeftMouseButton = true;
        }
        else if (SwingUtilities.isRightMouseButton(event)) {
            this.RightMouseButton = true;
        }
    }

    public Point getPoint() {
        this.LeftMouseButton = false;
        return new Point(this.koX, this.koY);
    }

    public Boolean isLeftMouseButton() {
        return this.LeftMouseButton;
    }
}
