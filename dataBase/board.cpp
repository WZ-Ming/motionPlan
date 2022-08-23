#include "board.h"
#include "ui_board.h"

board::board(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::board)
{
    ui->setupUi(this);
}

board::~board()
{
    delete ui;
}
