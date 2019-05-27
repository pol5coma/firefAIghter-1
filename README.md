# firefAIghter

Projecte de l'assignatura Robòtica Llenguatge i Planificació.

##################################################################

En aquest projecte trobem tots els arxius necessaris per a poder executar el programa que dirigeix el nostre robot. Aquest robot té l'objetiu de reconeixer un conjunt de zones d'una empresa, una industria.. I un cop sap moure's per aquests, quan rep l'ordre d'incendi, va fins la zona concreta, es situa devant del foc i l'apaga.

Les funcionalitats descrites anteriorment tenen origen tècnic en l'assignatura de Visió per Computador. Tant el detector de linies self._linefollower() i els detectors de foc self._LMC() i self._NeuralFireNet() s'han implementat aplicant tècniques de visió les quals ens han ajudat a arribar als resultats esperats.

###################################################################

Per a executar el codi:

if op == '1': # Execució del seguidor de linies self._lineFollower(op='store') elif op == '2': # Execució del seguidor de linies i detector de foc self._lineFollower(op='goto',zone=0) #self.identifyWhiteZones() elif op =='3': # Execució del LMC (Luminicense Motion Color detection) self.identifyWhiteZones() elif op == '4': # Disparar aigua sobre el foc self._shootWater()
