//
// Created by artem-kondratew on 05.03.23.
//

#ifndef OMEGABOT_GRAPHICS_H
#define OMEGABOT_GRAPHICS_H


#include "header.h"
#include "Connect.hpp"


#define KEY_RETURN     10

#define ID_X            9
#define GOAL_X         17
#define ANGLE_X        23
#define SPEED_X        30
#define TORQUE_X       37
#define IS_MOVING_X    45
#define EDGE_X         55

#define COMMAND_Y       8
#define LAST_COMMAND_Y  9

namespace Graphics {

    int CURS_Y = 0;
    int CURS_X = 0;


    int get_columns() {
        struct winsize window{};
        ioctl(0, TIOCGWINSZ, &window);
        return window.ws_col;
    }


    int get_rows() {
        struct winsize window{};
        ioctl(0, TIOCGWINSZ, &window);
        return window.ws_row;
    }


    void finish() {
        Connect::disconnectArduino();
        resetty();
        endwin();
        exit(0);
    }


    void sighandler(int sig) {
        if (sig == SIGINT) {
            finish();
        }
    }


    void print_table() {
        for (int i = 1; i < 5; i++) {
            move(i + 1, 0);
            printw("servo%d", i);
        }

        move(0, ID_X);
        printw("id");

        move(0, GOAL_X);
        printw("goal");
        move(0, ANGLE_X);
        printw("angle");

        move(0, SPEED_X);
        printw("speed");

        move(0, TORQUE_X);
        printw("torque");

        move(0, IS_MOVING_X);
        printw("is_moving");

        move(COMMAND_Y - 1, 0);
        printw("Set command:");

        for (int i = 0; i < 7; i++) {
            move(i, EDGE_X);
            printw("|");
        }

        refresh();
    }


    void print_id() {
        for (int i = 1; i < 5; i++) {
            move(i + 1, ID_X + 1);
            printw("%d", i);
        }
        refresh();
    }


    void print_exit_option() {
        move(get_rows() - 1, 0);
        printw("Press 'Ctrl+C' to exit");
        refresh();
    }


    void init_graphics() {
        initscr();
        savetty();  //  save terminal settings

        //nonl();  //  deny going to the new line

        cbreak();  //  send buffer after pressing enter
        echo();  //  visible printing
        timeout(0);

        //leaveok(stdscr, TRUE);
        //curs_set(0);  //  hide cursor
        keypad(stdscr, TRUE);

        //signal(SIGINT, sighandler);  moved to different thread

        clear();
        print_table();
        print_id();
        print_exit_option();
        move(COMMAND_Y, 0);
        refresh();
    }


    void key_left_proc() {
        Connect::resetCommand();
        Connect::turnLeft();
    }


    void key_right_proc() {
        Connect::resetCommand();
        Connect::turnRight();
    }


    void key_up_proc() {
        Connect::resetCommand();
        Connect::moveForward();
    }


    void key_down_proc() {
        Connect::resetCommand();
        Connect::moveBackward();
    }


    void key_b_proc() {
        Connect::resetCommand();
        Connect::beep();
    }


    void key_l_proc() {
        Connect::resetCommand();
        Connect::blink();
    }


    void key_proc() {
        while (true) {
            int key = getch();

            if (key == KEY_LEFT) {
                key_left_proc();
                continue;
            }
            if (key == KEY_RIGHT) {
                key_right_proc();
                continue;
            }
            if (key == KEY_UP) {
                key_up_proc();
                continue;
            }
            if (key == KEY_DOWN) {
                key_down_proc();
                continue;
            }
            if (key == ERR) {
                continue;
            }
        }
    }

}


#endif //OMEGABOT_GRAPHICS_H
