import Robonomics from 'robonomics-js'
import Provider from './provider'

let robonomics = null
const getRobonomics = (lighthouse) => {
  if (robonomics === null) {
    // const socket = io('http://localhost:9999')
    const socket = io('https://wss.pool.aira.life')
    robonomics = new Robonomics({
      web3,
      provider: new Provider(socket),
      lighthouse,
      // ens: '0x7390F2ec825E6028d98349F7893Fbe32051205Fa' // my private
      ens: '0xA4e0cD190D967540BA1756316838827CE41B8da5' // kovan
      // ens: '0xd4F9C19B32e57cC2485877Ef9EBA8CeAA91687b2', // ropsten
    })
  }
  return robonomics
}

export default getRobonomics
