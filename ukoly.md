## Úkoly 

* připojit IR 
https://laskarduino.cz/vstupni-periferie-cidla/131012-ir-opticky-infra-snimac-fc-51.html
* držák na mobil - robot ho poveze s sebou a bude do něj psát log 
* přímý výpis hodnot enkodérů 
* horni panel
* držák na majáček 
* cvičný robot 
* nachystat program pro cervenou a modrou stranu 
*na tlacitka zapojit pullupy  

### Nespěchá 


 příklad na tlačítka a LED  

## Vize 

co možná jeden algoritmus Bratislava i Praha -> z toho asi nic nebude 


## Algoritmus 

1. pojedu rychle
2. trasa: b3, c2, c1  (dříve než soupeř), b1, b5, přitom si pamatuju, kde byly plechovky   
    * pokud to nestihnu dříve než soupeř, tak zpět na b3, potom c4, c5, b5, b1, a1, vyložím plechovky, A3
3. na A5, otočka směr A4, vyložím plechovky, A3
4.  podle toho, kde byly plechovky, zvolím z tabulky trasu pro další objezd, tu pak jezdím pořád dokola  
5. cíl je 7 plechovek (nebo radši 9 ?)


## Algoritmus - varianta 

- projet S (8 ultrazvuků )
- vybrat kostky dané barvy (pixy ) a naložit 
- odvézt do cílového umístění (dva barevné senzory )
- vrátit se zpět 

#### Další 

 mít definované pole, kde zastavím a počkám na konec  -> podle soupeře, výchozí je A3
 pro jistotu také časovač na 3 minuty (bez pár sekund), který vypne motory a spadne do Kubovy pasti 
detekce pozic plechovek pomocí IR 
pohyb o daný počet mříží vpřed i vzad 
otočka o 45, 90 stupňů 


#### Poznámky

-----------
1. buď je alespoň 1 plechovka v řade B 
2. nebo jsou všechny v řadě C
--------------


co dělat, když detekuje robota=soupeře ?  a na jakou vzdálenost ho má detekovat? 
co dělat, když chytí plechovku ? 

## Stavový automat 

* vědět, kde je a jak je otočený 
* může to ověřovat na T a na rohu 
* na T se může srovnat 



## Postup 

* QRD číst 
* IR senzory - mají v sobě komparátor -> číst přes digitalRead(); na kterémkoli pinu 
* ultrazvuk? 
* kalibrace senzorů 
* zatočit o 90 stupňů
* jedu (skoro) rovně
* funkce, která vrací kde pod lištou přední/zadní je čára (pod kterým senzorem, číslo od -50 do 50, druhý parametr - indikace křižovatek)
* srovnání se na čáře 
* uvnitř procedury na rychlost postupné zrychlování a zpomalování ? 


## Poznámky všeobecné

výška není omezená v Praze ani v Bratislavě 
v Bratislavě se jezdí 3 minuty 

u serva je nějaká chyba, která způsobuje, že nestačí zapnout attach, ale je potřeba poslat do serva první pokyn, aby začal attach posílat do serva signál - ověřit

ověřit kód pro blikání LED přes expandér 



