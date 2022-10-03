# NTUST-1101EmbeddedSystemDesignLab_Lab4
NTUST 110-1-溫進坤-嵌入式系統設計實習 Lab4(踩地雷遊戲)

##### 作業題目：
>  1. 開機後自動進入設定畫面，設定炸彈數量5 - 99，設定完成後開始遊戲。
>  2. 炸彈需”隨機”放置於15*8的畫面中。
>  3. 遊戲中左上角顯示炸彈總數量。
>  4. 遊戲中右上方顯示遊戲開始時間(分:秒)。
>  5. 點擊的點周邊八個點若有炸彈，顯示炸彈數量。
>  6. 點擊的點不是炸彈或炸彈周圍八個點，顯示空白，並且把相連結的空白部分全部顯示出來，直到周邊有炸彈的點。
>  7. 畫面中只剩下炸彈點未點開時顯示Win ，遊戲時間停止。
>  8. 點擊到炸彈點後顯示Fail ，並且顯示所有炸彈位置，遊戲時間停止。
>  9. 遊戲結束後，按下USR 按鍵進入設定畫面。
>  10.使用STemWin或TouchGFX完成GUI畫面。

##### 改進功能：
>  1. 改為使用LVGL製作GUI與自行porting。(修改題目5.)

##### 使用環境：
>  1. STM32CubeMX Version 6.4.0
>  2. Keil µVision V5.36.0.0

#### 執行結果：
![](https://i.imgur.com/8IuN0Zd.jpg)
![](https://i.imgur.com/wamN9LD.jpg)
![](https://i.imgur.com/9YmiwQ3.jpg)
![](https://i.imgur.com/dLhtLtv.jpg)
