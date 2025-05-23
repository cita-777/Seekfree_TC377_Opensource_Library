/**
 * *****************************************************************************
 * @file        menu_core.h
 * @brief
 * @author      cita (juricek.chen@gmail.com)
 * @date        2025-01-31
 * @copyright   cita
 * *****************************************************************************
 */

#ifndef MENU_CORE_H
#define MENU_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/

/*-----------------------------------macro------------------------------------*/
#define COUNT_OF(A) (sizeof(A) / sizeof(*(A)))
#define MAX_MENU_LEVELS 4
/*----------------------------------typedef-----------------------------------*/
typedef void (*tMenuFunc)(void);
typedef struct sMenuItem *tMenuItem;
typedef struct sMenu     *tMenu;
/*�˵���*/
struct sMenuItem
{
    const char *pszTitle;           // �˵�ҳ����
    tMenuFunc   pfMenuFunc;         // ���ܺ���
    tMenuFunc   pfUpDownMenuFunc;   // ���¶���ִ�еĹ���
    tMenu       psSubMenu;          // �Ӳ˵�
};

/*�˵�ҳ*/
struct sMenu
{
    const char *pszTitle;   // �˵�ҳ����
    tMenuItem   psItems;    // �˵�ҳ�а����Ĳ˵���
    uint8       nItems;     // �˵�����
};
typedef struct
{
    int x;
    int y;
} Point;
/*----------------------------------variable----------------------------------*/
/*-------------------------------------os-------------------------------------*/
/*----------------------------------function----------------------------------*/
extern void Menu_Init(void);
extern void DisplayMenu(void);
extern void SelFunc(void);
extern void ReturnFunc(void);
extern void UpFunc(void);
extern void DownFunc(void);
extern void MainMenuInit(void);
extern void MenuTask(void);
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* MENU_CORE_H */
