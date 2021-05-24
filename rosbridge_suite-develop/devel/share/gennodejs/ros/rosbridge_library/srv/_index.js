
"use strict";

let TestResponseOnly = require('./TestResponseOnly.js')
let AddTwoInts = require('./AddTwoInts.js')
let TestRequestOnly = require('./TestRequestOnly.js')
let TestMultipleRequestFields = require('./TestMultipleRequestFields.js')
let SendBytes = require('./SendBytes.js')
let TestMultipleResponseFields = require('./TestMultipleResponseFields.js')
let TestEmpty = require('./TestEmpty.js')
let TestRequestAndResponse = require('./TestRequestAndResponse.js')
let TestNestedService = require('./TestNestedService.js')
let TestArrayRequest = require('./TestArrayRequest.js')

module.exports = {
  TestResponseOnly: TestResponseOnly,
  AddTwoInts: AddTwoInts,
  TestRequestOnly: TestRequestOnly,
  TestMultipleRequestFields: TestMultipleRequestFields,
  SendBytes: SendBytes,
  TestMultipleResponseFields: TestMultipleResponseFields,
  TestEmpty: TestEmpty,
  TestRequestAndResponse: TestRequestAndResponse,
  TestNestedService: TestNestedService,
  TestArrayRequest: TestArrayRequest,
};
