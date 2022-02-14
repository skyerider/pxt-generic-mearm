function reset()  {
    mearm.moveToCentre(mearm.MearmServo.Base)
    mearm.moveToCentre(mearm.MearmServo.Right)
    mearm.moveToCentre(mearm.MearmServo.Left)
    mearm.moveToCentre(mearm.MearmServo.Grip)
}
input.onButtonPressed(Button.B, () => {
    mearm.closeGrip()
})
input.onButtonPressed(Button.A, () => {
    mearm.openGrip()
})
input.onButtonPressed(Button.AB, () => {
    reset()
})
reset()
basic.forever(() => {

})
