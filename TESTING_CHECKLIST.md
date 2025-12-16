# Testing Checklist - Modern Modal-Based Authentication

Use this checklist to verify your authentication implementation is working correctly.

## ✅ Pre-Testing Setup

- [ ] Auth server is running (`cd auth-server && npm run dev`)
- [ ] Docusaurus is running (`npm start`)
- [ ] Both servers started without errors
- [ ] Browser is open to http://localhost:3000

## ✅ Test 1: Protected Route Access (Unauthenticated)

**Steps:**
1. [ ] Open incognito/private browser window
2. [ ] Navigate to: http://localhost:3000/docs/advanced-topics
3. [ ] Wait for page to load

**Expected Results:**
- [ ] Page loads without errors
- [ ] Signup modal automatically opens
- [ ] Modal shows title: "Start Your Learning Journey"
- [ ] Context message displays: "Sign up to unlock Advanced Topics..."
- [ ] Lock icon (🔒) visible on page behind modal
- [ ] Modal has email, name, and password fields
- [ ] "Already have an account? Sign In" link is visible

**If this fails:**
- Check browser console for errors
- Verify ProtectedRoute is imported in advanced-topics.md
- Check auth server is running

---

## ✅ Test 2: User Signup

**Steps:**
1. [ ] Fill in signup form:
   - Name: "Test User"
   - Email: "test@example.com"
   - Password: "password123"
2. [ ] Click "Sign Up" button
3. [ ] Wait for processing

**Expected Results:**
- [ ] Button shows "Creating account..." during processing
- [ ] No error messages appear
- [ ] Modal closes automatically
- [ ] Page redirects to /docs/advanced-topics
- [ ] Welcome banner appears at top
- [ ] Welcome banner shows: "Good morning/afternoon/evening, Test!"
- [ ] Full page content is now visible (not locked)

**If this fails:**
- Check browser console for errors
- Check network tab for failed requests
- Verify auth server is reachable
- Check database connection in auth server

---

## ✅ Test 3: Redirect Verification

**Steps:**
1. [ ] Sign out (click user menu → Sign Out)
2. [ ] Navigate to: http://localhost:3000/docs/advanced-topics?section=test
3. [ ] Note the URL has query parameter: `?section=test`
4. [ ] Signup modal opens
5. [ ] Sign up with new email (e.g., "test2@example.com")

**Expected Results:**
- [ ] After signup, URL is: /docs/advanced-topics?section=test
- [ ] Query parameter `?section=test` is preserved
- [ ] User is on the exact page they tried to access

**If this fails:**
- Check localStorage for 'auth_intended_route' key
- Verify redirect logic in LoginModal.jsx and SignupModal.jsx
- Check browser console for errors

---

## ✅ Test 4: Login (Existing User)

**Steps:**
1. [ ] Sign out if signed in
2. [ ] Navigate to: http://localhost:3000/docs/advanced-topics
3. [ ] Signup modal opens
4. [ ] Click "Already have an account? Sign In"
5. [ ] Fill in login form:
   - Email: "test@example.com"
   - Password: "password123"
6. [ ] Click "Sign In"

**Expected Results:**
- [ ] Modal switches from signup to login
- [ ] Login modal title: "Welcome Back"
- [ ] Context message appears
- [ ] After login, redirected to /docs/advanced-topics
- [ ] Welcome banner appears
- [ ] Content is visible

**If this fails:**
- Verify user exists in database
- Check password is correct
- Check auth server logs
- Verify session cookie is set

---

## ✅ Test 5: Progress Tracking

**Steps:**
1. [ ] Ensure you're signed in
2. [ ] Visit: http://localhost:3000/docs/intro
3. [ ] Visit: http://localhost:3000/docs/ros2-intro
4. [ ] Visit: http://localhost:3000/docs/advanced-topics
5. [ ] Check welcome banner

**Expected Results:**
- [ ] Welcome banner updates to show last visited page
- [ ] Banner shows: "Continue where you left off: [page title]"
- [ ] Clicking the link takes you to that page

**To verify localStorage:**
- [ ] Open browser DevTools → Application → localStorage
- [ ] Find key: `user_progress`
- [ ] Value should be JSON with `lastVisited` and `visitedPages`

**If this fails:**
- Check localStorage permissions
- Verify trackPageVisit is being called
- Check browser console for errors

---

## ✅ Test 6: Welcome Banner Display

**Steps:**
1. [ ] Ensure you're signed in
2. [ ] Navigate to: http://localhost:3000/docs/advanced-topics

**Expected Results:**
- [ ] Welcome banner appears at top of page
- [ ] Banner has gradient background (primary color)
- [ ] Shows time-based greeting:
  - Before 12pm: "Good morning"
  - 12pm-6pm: "Good afternoon"
  - After 6pm: "Good evening"
- [ ] Shows first name from signup
- [ ] If progress exists, shows "Continue where you left off"

**If this fails:**
- Verify WelcomeBanner is imported in the doc
- Check user is authenticated
- Check userProgress in AuthContext

---

## ✅ Test 7: Switch Between Modals

**Steps:**
1. [ ] Sign out
2. [ ] Navigate to protected page
3. [ ] Signup modal opens
4. [ ] Click "Already have an account? Sign In"
5. [ ] Login modal should open
6. [ ] Click "Don't have an account? Sign Up"
7. [ ] Signup modal should open again

**Expected Results:**
- [ ] Modals switch smoothly
- [ ] No page reload
- [ ] Form fields are empty when switching
- [ ] No errors in console

---

## ✅ Test 8: Close Modal Behavior

**Steps:**
1. [ ] Ensure signed out
2. [ ] Navigate to protected page
3. [ ] Modal opens
4. [ ] Click X button (top right)
5. [ ] Click outside modal (on overlay)

**Expected Results:**
- [ ] Both actions close the modal
- [ ] Page content remains (lock icon still visible)
- [ ] Can re-open modal by clicking buttons on page

---

## ✅ Test 9: Public Page Access

**Steps:**
1. [ ] Sign out
2. [ ] Navigate to: http://localhost:3000/docs/intro

**Expected Results:**
- [ ] Page loads without modal
- [ ] Content is immediately visible
- [ ] No authentication required
- [ ] Can navigate freely

---

## ✅ Test 10: Social Login (Optional)

**Note:** Only test if you've configured Google/GitHub OAuth

**Steps:**
1. [ ] Sign out
2. [ ] Navigate to protected page
3. [ ] Modal opens
4. [ ] Click "Google" or "GitHub" button

**Expected Results:**
- [ ] OAuth provider window opens
- [ ] Can complete authentication
- [ ] Redirected back to intended page
- [ ] User is authenticated

**If this fails:**
- Verify OAuth credentials in auth-server/.env
- Check redirect URLs are configured
- Check auth provider dashboard

---

## ✅ Test 11: Mobile Responsiveness

**Steps:**
1. [ ] Open browser DevTools
2. [ ] Switch to mobile view (Toggle device toolbar)
3. [ ] Test viewport: iPhone 12 Pro (390x844)
4. [ ] Navigate to protected page
5. [ ] Modal should open

**Expected Results:**
- [ ] Modal is responsive
- [ ] All text is readable
- [ ] Buttons are touch-friendly
- [ ] No horizontal scrolling
- [ ] User name hidden in navbar (mobile)
- [ ] Modal fits on screen

---

## ✅ Test 12: Dark Mode

**Steps:**
1. [ ] Toggle dark mode in Docusaurus
2. [ ] Navigate to protected page
3. [ ] Open auth modal

**Expected Results:**
- [ ] Modal has dark theme
- [ ] Text is readable (light on dark)
- [ ] Inputs have dark styling
- [ ] Welcome banner adapts to dark mode
- [ ] All components support dark mode

---

## ✅ Test 13: Error Handling

**Test 13.1: Wrong Password**
1. [ ] Try to login with wrong password
2. [ ] Expected: Error message appears "Invalid credentials" or similar

**Test 13.2: Existing Email**
1. [ ] Try to signup with existing email
2. [ ] Expected: Error message appears

**Test 13.3: Short Password**
1. [ ] Try to signup with password < 8 chars
2. [ ] Expected: Error message "Password must be at least 8 characters"

**Test 13.4: Auth Server Down**
1. [ ] Stop auth server
2. [ ] Try to login/signup
3. [ ] Expected: Error message appears
4. [ ] Start auth server again

---

## ✅ Test 14: Session Persistence

**Steps:**
1. [ ] Sign in
2. [ ] Close browser tab
3. [ ] Open new tab
4. [ ] Navigate to: http://localhost:3000/docs/advanced-topics

**Expected Results:**
- [ ] User is still authenticated
- [ ] No modal appears
- [ ] Content is visible
- [ ] Welcome banner shows

**If this fails:**
- Check cookies are enabled
- Verify httpOnly cookies are set
- Check session expiration in auth server

---

## ✅ Test 15: Sign Out

**Steps:**
1. [ ] Ensure signed in
2. [ ] Click user menu (top right)
3. [ ] Click "Sign Out"
4. [ ] Try to access protected page

**Expected Results:**
- [ ] User menu closes
- [ ] User is signed out
- [ ] Protected pages now show modal
- [ ] Session cookie is cleared

---

## ✅ Test 16: Multiple Protected Pages

**Steps:**
1. [ ] Sign out
2. [ ] Try to access /docs/advanced-topics
3. [ ] Note the context message
4. [ ] Close modal
5. [ ] Create another protected page with different message

**Expected Results:**
- [ ] Each page can have custom context message
- [ ] Redirect works for any protected page
- [ ] Progress tracking works across all pages

---

## ✅ Performance Tests

**Test 17.1: Load Time**
- [ ] Protected page loads within 2 seconds
- [ ] Modal appears within 500ms
- [ ] No console warnings about performance

**Test 17.2: Network Requests**
- [ ] Check Network tab in DevTools
- [ ] Session check happens on page load
- [ ] No unnecessary repeated requests
- [ ] Auth requests complete within 2 seconds

---

## 🐛 Common Issues & Solutions

### Issue: Modal doesn't open
**Solution:**
- Verify ProtectedRoute is imported
- Check MDX syntax in doc file
- Check browser console for errors
- Ensure auth server is running

### Issue: Redirect not working
**Solution:**
- Check localStorage in DevTools
- Verify 'auth_intended_route' key exists
- Check redirect logic in modal files
- Clear localStorage and try again

### Issue: Progress not saving
**Solution:**
- Check localStorage permissions
- Verify user is authenticated
- Check trackPageVisit is called
- Clear localStorage and try again

### Issue: Session not persisting
**Solution:**
- Enable cookies in browser
- Check auth server is setting cookies
- Verify credentials: 'include' in requests
- Check CORS configuration

---

## ✅ Final Verification

After completing all tests:

- [ ] All core features work (auth, redirect, progress)
- [ ] No errors in browser console
- [ ] No errors in auth server logs
- [ ] Mobile view works correctly
- [ ] Dark mode works correctly
- [ ] Error messages are user-friendly
- [ ] Performance is acceptable

## 🎉 Testing Complete!

If all checks passed, your Modern modal-based authentication is fully functional and ready for production!

**Next Steps:**
1. Document any customizations you made
2. Test with real users
3. Deploy to production
4. Monitor error logs

---

**Testing Date:** ___________
**Tester:** ___________
**All Tests Passed:** [ ] Yes [ ] No
**Notes:** ___________________________________________
