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
/*菜单项*/
struct sMenuItem
{
    const char *pszTitle;           // 菜单页标题
    tMenuFunc   pfMenuFunc;         // 功能函数
    tMenuFunc   pfUpDownMenuFunc;   // 上下动作执行的功能
    tMenu       psSubMenu;          // 子菜单
};

/*菜单页*/
struct sMenu
{
    const char *pszTitle;   // 菜单页标题
    tMenuItem   psItems;    // 菜单页中包含的菜单项
    uint8       nItems;     // 菜单项数
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
