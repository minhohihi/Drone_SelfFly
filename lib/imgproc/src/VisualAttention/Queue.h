//#ifndef BQueue_
//#define BQueue_
#define MAX_QUEUE_SIZE 500
#include <stdlib.h>
#include <stdio.h>

typedef struct {
    //int c;
    int row,col;
    //int L_count;
    //int left_x, right
} element;

// queue �����. 
//typedef struct queue *queue_pointer;
typedef struct queue {
    element item;
    struct queue * link; // �ڱ� ����... ���� ���� �����Ѿ� �ϴϱ�..
} *queue_pointer;

void addq(queue_pointer *front, queue_pointer *rear, element item)
{
   queue_pointer temp = (queue_pointer) malloc(sizeof(queue));
   temp->item = item; // �߰��Ǵ� ���� ���Ұ� 
   temp->link = NULL; // ���簡 ������ �̹Ƿ�...
   if (*front) // ó�� ������ ���� �ƴ϶��.. 
       (*rear)->link = temp;
   else *front = temp; // ���� ó�� �����������..
   *rear = temp;
}

element deleteq(queue_pointer *front, queue_pointer *rear)
{
    queue_pointer temp=*front;
    element item;
    item.row = -1;
    if (*front==NULL)
    {
        //free(rear2);
        *rear = NULL;
        //printf ("\n queue is empty");
        return item;
    }
    item = temp->item; // ó���� ������ �ϴϱ�..
    *front = temp->link; // ������ �������ְ�..
    free(temp); // �����.
    return item; // �������� �� ��ȯ�Ѵ�.
}
//#endif
