function Provider(socket) {
  this.socket = socket;
}
Provider.prototype.ready = function() {
  this.socket.emit('chanel', 'chanel');
  return new Promise((resolve) => {
    resolve();
  });
};
Provider.prototype.push = function(chanel, msg) {
  return new Promise((resolve) => {
    this.socket.emit(chanel, JSON.stringify(msg));
    resolve();
  });
};
Provider.prototype.watch = function(chanel, cb) {
  this.socket.emit('chanel', chanel);
  this.socket.on(chanel, (msg) => {
    cb(msg);
  });
};

var robonomics, chanel, message;

function formatXRTDecimals (amount) {
  return (new web3.BigNumber(amount)).shift(-9).toNumber();
}

function sendApprove() {
    robonomics.xrt.send('approve', [robonomics.address.factory, 1000000000], { from: web3.eth.accounts[0] })
    .then((result) => {
        console.log(result)
    });
}

function sendAsk() {
  web3.eth.getBlock('latest', (e, r) => {
    var ask = message.create({
      model: 'QmWboFP8XeBtFMbNYK3Ne8Z3gKFBSR5iQzkKgeNgQz3dZ4',
      objective: 'QmUo3vvSXZPQaQWjb3cH3qQo1hc8vAUqNnqbdVABbSLb6r',
      token: robonomics.address.xrt,
      cost: 1,
      count: 1,
      validator: '0x0000000000000000000000000000000000000000',
      validatorFee: 0,
      deadline: r.number + 1000
    });
    message.sign(web3.eth.accounts[0], ask)
      .then(() => chanel.push(ask))
      .then(() => {
        console.log('ok');
      });
  });
}

function app() {
  var socket = io('https://wss.pool.aira.life');
  var signer = (account, hash) => Promise.promisify(web3.eth.sign)(account, hash);
  var lighthouse = '0x36d5D6C5BB8fFDC30e2699Bb1E397Fb48aC6CE5d';

  robonomics = new Robonomics.default({
    web3,
    provider: new Provider(socket),
  });
  message = robonomics.message(signer);
  chanel = robonomics.chanel(lighthouse);

  chanel.asks((msg) => {
    const acc = msg.recover();
    $('#ask').append("<div style='margin-bottom: 10px;border-bottom: 1px solid #eee;'>" +
      "<ul>" +
        "<li><b>account: </b>" + acc + "</li>" +
        "<li><b>model: </b><a href='https://ipfs.io/ipfs/" + msg.model + "' target='_blank'>" + msg.model + "</a></li>" +
        "<li><b>objective: </b><a href='https://ipfs.io/ipfs/" + msg.objective + "' target='_blank'>" + msg.objective + "</a></li>" +
        "<li><b>token: </b>" + msg.token + "</li>" +
        "<li><b>cost: </b>" + msg.cost + "</li>" +
        "<li><b>count: </b>" + msg.count + "</li>" +
        "<li><b>validatorFee: </b>" + msg.validatorFee + "</li>" +
        "<li><b>deadline: </b>" + msg.deadline + "</li>" +
      "</ul>" +
    "</div>");
  });

  chanel.bids((msg) => {
    const acc = msg.recover();
    $('#bid').append("<div style='margin-bottom: 10px;border-bottom: 1px solid #eee;'>" +
      "<ul>" +
        "<li><b>account: </b>" + acc + "</li>" +
        "<li><b>model: </b><a href='https://ipfs.io/ipfs/" + msg.model + "' target='_blank'>" + msg.model + "</a></li>" +
        "<li><b>token: </b>" + msg.token + "</li>" +
        "<li><b>cost: </b>" + msg.cost + "</li>" +
        "<li><b>count: </b>" + msg.count + "</li>" +
        "<li><b>lighthouseFee: </b>" + msg.validatorFee + "</li>" +
        "<li><b>deadline: </b>" + msg.deadline + "</li>" +
      "</ul>" +
    "</div>");
  });

  robonomics.factory.watchLiability((liability, result) => {
    web3.eth.getTransaction(result.transactionHash, (e, r) => {
      if (r.to.toLowerCase() === lighthouse.toLowerCase()) {
        liability.getInfo()
          .then((info) => {
            $('#liability').append("<div style='margin-bottom: 10px;border-bottom: 1px solid #eee;'>" +
              "<ul>" +
                "<li><b>liability: </b><a href='https://kovan.etherscan.io/address/" + liability.address + "' target='_blank'>" + liability.address + "</a></li>" +
                "<li><b>lighthouse: </b><a href='https://kovan.etherscan.io/address/" + r.to + "' target='_blank'>" + r.to + "</a></li>" +
                "<li><b>from: </b><a href='https://kovan.etherscan.io/address/" + r.from + "' target='_blank'>" + r.from + "</a></li>" +
                "<li><b>model: </b><a href='https://ipfs.io/ipfs/" + info.model + "' target='_blank'>" + info.model + "</a></li>" +
                "<li><b>objective: </b><a href='https://ipfs.io/ipfs/" + info.objective + "' target='_blank'>" + info.objective + "</a></li>" +
                "<li><b>token: </b>" + info.token + "</li>" +
                "<li><b>promisee: </b>" + info.promisee + "</li>" +
                "<li><b>promisor: </b>" + info.promisor + "</li>" +
              "</ul>" +
            "</div>");

	    $('#form_post').show();
          });
      }
    });
  });
}

window.addEventListener('load', () => {
  if (typeof web3 !== 'undefined') {
    app()

    robonomics.xrt.call('balanceOf', [web3.eth.accounts[0]])
    .then((balance) => {
        console.log('My balance', formatXRTDecimals(balance))
        $('#xrt_balance').html(formatXRTDecimals(balance))
    });

    robonomics.xrt.call('allowance', [web3.eth.accounts[0], robonomics.address.factory])
    .then((amount) => {
        console.log('My approve', formatXRTDecimals(amount))
        if(amount > 0) {
            $('#approve_btn').prop('disabled', true);
        }
    });

    console.log('ok');
  } else {
    console.error('not web3');
  }
});
